use std::error::Error;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use geometry_msgs::msg::Twist;
use sensor_msgs::msg::BatteryState;
use rclrs::{Context, create_node, spin, QOS_PROFILE_DEFAULT};
use rppal::i2c::I2c;

// =========== I2C and Motor Constants ===========
const I2C_BUS_NUMBER: u8 = 1;   // Adjust if Pi 5 enumerates differently
const MOTOR_ADDR: u16 = 0x34;

// Addresses from your working Python script
const ADC_BAT_ADDR: u8  = 0x00;  // 2 bytes => battery in millivolts
const MOTOR_TYPE_ADDR: u8             = 0x14;
const MOTOR_ENCODER_POLARITY_ADDR: u8 = 0x15;
const MOTOR_FIXED_SPEED_ADDR: u8      = 0x33;

// Example motor/polarity
const MOTOR_TYPE_JGB37_520_12V_110RPM: u8 = 3;
const MOTOR_ENCODER_POLARITY: u8          = 0;

// =========== Motor Driver ===========
struct MotorDriver {
    i2c: Mutex<I2c>,
}

impl MotorDriver {
    /// Attempt I2C open + configure. If board is off, returns Err.
    fn try_new() -> Result<Self, Box<dyn Error>> {
        let mut i2c = I2c::with_bus(I2C_BUS_NUMBER)?;
        i2c.set_slave_address(MOTOR_ADDR)?;

        // Write motor type/polarity
        i2c.write(&[MOTOR_TYPE_ADDR, MOTOR_TYPE_JGB37_520_12V_110RPM])?;
        i2c.write(&[MOTOR_ENCODER_POLARITY_ADDR, MOTOR_ENCODER_POLARITY])?;

        println!("MotorDriver connected: bus={}, addr=0x{:X}", I2C_BUS_NUMBER, MOTOR_ADDR);

        Ok(Self {
            i2c: Mutex::new(i2c),
        })
    }

    /// Set M1, M2 speeds in [-100..100]. 
    /// We'll send [REGISTER, left, right].
    fn set_tracks(&self, left: i8, right: i8) {
        println!("set_tracks: left={left}, right={right}");
        let data = [
            MOTOR_FIXED_SPEED_ADDR,
            left as u8,
            right as u8,
        ];

        let mut bus = self.i2c.lock().unwrap();
        if let Err(e) = bus.write(&data) {
            eprintln!("I2C write error: {e}");
        }
    }

    /// Read battery voltage as millivolts (u16).
    fn read_battery_mv(&self) -> Result<u16, Box<dyn Error>> {
        let mut bus = self.i2c.lock().unwrap();

        // Indicate which register:
        bus.write(&[ADC_BAT_ADDR])?;

        let mut buf = [0u8; 2];
        bus.read(&mut buf)?;

        let mv = ((buf[1] as u16) << 8) | (buf[0] as u16);
        Ok(mv)
    }
}

// On Drop => set speeds=0
impl Drop for MotorDriver {
    fn drop(&mut self) {
        println!("MotorDriver drop => setting speeds=0");
        let data = [
            MOTOR_FIXED_SPEED_ADDR,
            0u8,
            0u8,
        ];
        if let Ok(mut bus) = self.i2c.lock() {
            let _ = bus.write(&data);
        }
    }
}

// Convert Twist -> (left,right) in [-100..100]
fn twist_to_tracks(lin_x: f64, ang_z: f64) -> (i8, i8) {
    let linear_scale = 60.0;
    let angular_scale = 40.0;

    let left = lin_x * linear_scale - ang_z * angular_scale;
    let right = lin_x * linear_scale + ang_z * angular_scale;

    let clamp = |v: f64| v.max(-100.0).min(100.0);
    (clamp(left) as i8, clamp(right) as i8)
}

fn main() -> Result<(), Box<dyn Error>> {
    // 1) Reconnect loop if board is off
    let driver = loop {
        match MotorDriver::try_new() {
            Ok(d) => break Arc::new(d),
            Err(e) => {
                eprintln!("MotorDriver not ready: {e}; retry in 3s...");
                thread::sleep(Duration::from_secs(3));
            }
        }
    };

    // 2) Start ROS 2
    let context = Context::new(std::env::args())?;
    let mut node = create_node(&context, "tank_control_node")?;

    // 3) Subscription to /cmd_vel => set tracks
    let sub = {
        let driver_sub = Arc::clone(&driver);
        node.create_subscription::<Twist, _>(
            "/cmd_vel",
            QOS_PROFILE_DEFAULT,
            move |msg: Twist| {
                let (left, right) = twist_to_tracks(msg.linear.x, msg.angular.z);
                driver_sub.set_tracks(left, right);
            },
        )?
    };

    // 4) Publisher for BatteryState on /battery
    let battery_pub = node.create_publisher::<BatteryState>(
        "/battery",
        QOS_PROFILE_DEFAULT,
    )?;

    // 5) Thread for reading battery every 5s, then publish
    {
        let driver_batt = Arc::clone(&driver);
        let battery_pub_cloned = battery_pub.clone();
        thread::spawn(move || {
            loop {
                match driver_batt.read_battery_mv() {
                    Ok(mv) => {
                        println!("Battery: {mv} mV");

                        // Fill all fields of BatteryState:
                        let mut msg = BatteryState::default();
                        
                        // We won't set a meaningful Header stamp in this snippet
                        // msg.header.stamp = ???

                        // 1) Voltage in Volts
                        msg.voltage = mv as f32 / 1000.0;

                        // 2) temperature, current, charge, capacity, percentage => NaN
                        msg.temperature = f32::NAN;
                        msg.current = f32::NAN;
                        msg.charge = f32::NAN;
                        msg.capacity = f32::NAN;
                        msg.percentage = f32::NAN;

                        // 3) design_capacity => 6.0 Ah
                        msg.design_capacity = 6.0;

                        // 4) status => 2 (Discharging)
                        // 5) health => 0 (Unknown)
                        // 6) technology => 3 (LiPo)
                        msg.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
                        msg.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
                        msg.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;

                        // 7) present => true
                        msg.present = true;

                        // 8) cell_voltage & cell_temperature => [NaN]
                        // If you have multiple cells, you can do so. We'll just do 1 cell.
                        msg.cell_voltage = vec![f32::NAN];
                        msg.cell_temperature = vec![f32::NAN];

                        // 9) location => "1", serial_number => "1"
                        msg.location = "1".to_owned();
                        msg.serial_number = "1".to_owned();

                        // Publish
                        battery_pub_cloned.publish(&msg).ok();
                    }
                    Err(e) => eprintln!("Battery read error: {e}"),
                }
                thread::sleep(Duration::from_secs(5));
            }
        });
    }

    println!("Node up. Subscribing on /cmd_vel, publishing BatteryState on /battery.");
    println!("Press Ctrl+C to exit. (Speeds=0 on exit.)");

    // 6) Spin forever
    spin(node)?;

    // On exit, sub & driver drop => speeds=0
    Ok(())
}

