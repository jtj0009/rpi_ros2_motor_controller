use std::error::Error;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use geometry_msgs::msg::Twist;
use sensor_msgs::msg::{BatteryState, JointState};
use rclrs::{Context, create_node, spin, QOS_PROFILE_DEFAULT};
use rppal::i2c::I2c;
use std_msgs::msg::Header; // from std_msgs crate
use builtin_interfaces::msg::Time; // from builtin_interfaces

// =========== I2C and Motor Constants ===========
const I2C_BUS_NUMBER: u8 = 1;   // Adjust if needed
const MOTOR_ADDR: u16 = 0x34;

// Register addresses
const ADC_BAT_ADDR: u8 = 0x00;  // 2 bytes: battery voltage in millivolts
const MOTOR_TYPE_ADDR: u8 = 0x14;
const MOTOR_ENCODER_POLARITY_ADDR: u8 = 0x15;
const MOTOR_FIXED_SPEED_ADDR: u8 = 0x33;

// New: Register address for reading the accumulated encoder counts for four motors.
// The Python code uses 0x3C (which is 60 in decimal).
const MOTOR_ENCODER_TOTAL_ADDR: u8 = 0x3C;

// Example motor type/polarity values
const MOTOR_TYPE_JGB37_520_12V_110RPM: u8 = 3;
const MOTOR_ENCODER_POLARITY: u8 = 0;

// =========== Motor Driver ===========
struct MotorDriver {
    i2c: Mutex<I2c>,
}

impl MotorDriver {
    /// Open and configure the I2C bus.
    fn try_new() -> Result<Self, Box<dyn Error>> {
        let mut i2c = I2c::with_bus(I2C_BUS_NUMBER)?;
        i2c.set_slave_address(MOTOR_ADDR)?;
        i2c.write(&[MOTOR_TYPE_ADDR, MOTOR_TYPE_JGB37_520_12V_110RPM])?;
        i2c.write(&[MOTOR_ENCODER_POLARITY_ADDR, MOTOR_ENCODER_POLARITY])?;
        println!("MotorDriver connected: bus={}, addr=0x{:X}", I2C_BUS_NUMBER, MOTOR_ADDR);
        Ok(Self {
            i2c: Mutex::new(i2c),
        })
    }

    /// Set motor speeds in the range [-100, 100]. Sends [REGISTER, left, right].
    fn set_tracks(&self, left: i8, right: i8) {
        println!("set_tracks: left={}, right={}", left, right);
        let data = [MOTOR_FIXED_SPEED_ADDR, left as u8, right as u8];
        let mut bus = self.i2c.lock().unwrap();
        if let Err(e) = bus.write(&data) {
            eprintln!("I2C write error: {}", e);
        }
    }

    /// Read battery voltage (millivolts).
    fn read_battery_mv(&self) -> Result<u16, Box<dyn Error>> {
        let mut bus = self.i2c.lock().unwrap();
        bus.write(&[ADC_BAT_ADDR])?;
        let mut buf = [0u8; 2];
        bus.read(&mut buf)?;
        let mv = ((buf[1] as u16) << 8) | (buf[0] as u16);
        Ok(mv)
    }

    /// Read encoder pulse counts.
    /// Reads 16 bytes from MOTOR_ENCODER_TOTAL_ADDR, with 4 bytes per motor (little-endian).
    fn read_encoders(&self) -> Result<[i32; 4], Box<dyn Error>> {
        let mut bus = self.i2c.lock().unwrap();
        // Write the register address for the total encoder counts.
        bus.write(&[MOTOR_ENCODER_TOTAL_ADDR])?;
        // Short delay to allow the controller to update the data.
        thread::sleep(Duration::from_millis(10));
        let mut buf = [0u8; 16];
        bus.read(&mut buf)?;
        let mut counts = [0i32; 4];
        for i in 0..4 {
            counts[i] = i32::from_le_bytes(buf[i*4..(i+1)*4].try_into()?);
        }
        Ok(counts)
    }
}

impl Drop for MotorDriver {
    fn drop(&mut self) {
        println!("MotorDriver drop => setting speeds=0");
        let data = [MOTOR_FIXED_SPEED_ADDR, 0u8, 0u8];
        if let Ok(mut bus) = self.i2c.lock() {
            let _ = bus.write(&data);
        }
    }
}

// Convert a Twist message to left/right motor commands.
fn twist_to_tracks(lin_x: f64, ang_z: f64) -> (i8, i8) {
    let linear_scale = 60.0;
    let angular_scale = 40.0;
    let left = lin_x * linear_scale - ang_z * angular_scale;
    let right = lin_x * linear_scale + ang_z * angular_scale;
    let clamp = |v: f64| v.max(-100.0).min(100.0);
    (clamp(left) as i8, clamp(right) as i8)
}

fn main() -> Result<(), Box<dyn Error>> {
    // 1) Loop until the motor driver is available.
    let driver = loop {
        match MotorDriver::try_new() {
            Ok(d) => break Arc::new(d),
            Err(e) => {
                eprintln!("MotorDriver not ready: {}; retry in 3s...", e);
                thread::sleep(Duration::from_secs(3));
            }
        }
    };

    // 2) Initialize ROS 2.
    let context = Context::new(std::env::args())?;
    let node = create_node(&context, "tank_control_node")?;

    // 3) Subscribe to /cmd_vel to set motor speeds.
    let _sub = {
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

    // 4) Publisher for battery voltage on /battery.
    let battery_pub = node.create_publisher::<BatteryState>("/battery", QOS_PROFILE_DEFAULT)?;
    // 5) Publisher for encoder counts on /encoder.
    let encoder_pub = node.create_publisher::<JointState>("/encoder", QOS_PROFILE_DEFAULT)?;

    // 6) Thread: Read battery voltage every 5 seconds and publish.
    {
        let driver_batt = Arc::clone(&driver);
        let battery_pub_cloned = battery_pub.clone();
        thread::spawn(move || {
            loop {
                match driver_batt.read_battery_mv() {
                    Ok(mv) => {
                        println!("Battery: {} mV", mv);
                        let mut batt_msg = BatteryState::default();
                        batt_msg.voltage = mv as f32 / 1000.0;
                        batt_msg.temperature = f32::NAN;
                        batt_msg.current = f32::NAN;
                        batt_msg.charge = f32::NAN;
                        batt_msg.capacity = f32::NAN;
                        batt_msg.percentage = f32::NAN;
                        batt_msg.design_capacity = 6.0;
                        batt_msg.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
                        batt_msg.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
                        batt_msg.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
                        batt_msg.present = true;
                        batt_msg.cell_voltage = vec![f32::NAN];
                        batt_msg.cell_temperature = vec![f32::NAN];
                        batt_msg.location = "1".to_owned();
                        batt_msg.serial_number = "1".to_owned();
                        battery_pub_cloned.publish(&batt_msg).ok();
                    }
                    Err(e) => eprintln!("Battery read error: {}", e),
                }
                thread::sleep(Duration::from_secs(5));
            }
        });
    }

    // 7) Thread: Read encoder counts every second and publish as JointState.
    {
        let driver_enc = Arc::clone(&driver);
        let encoder_pub_cloned = encoder_pub.clone();
        thread::spawn(move || {
            loop {
                match driver_enc.read_encoders() {
                    Ok(counts) => {
                        println!("Encoders: {:?}", counts);
                        // Build a JointState message with four motors.
                        let joint_msg = JointState {
                            header: Header {
                                stamp: Time { sec: 0, nanosec: 0 },
                                frame_id: "encoder".to_owned(),
                            },
                            name: vec![
                                "motor1".to_owned(),
                                "motor2".to_owned(),
                                "motor3".to_owned(),
                                "motor4".to_owned(),
                            ],
                            position: counts.iter().map(|&c| c as f64).collect(),
                            // Provide non-empty velocity and effort arrays.
                            velocity: vec![0.0, 0.0, 0.0, 0.0],
                            effort: vec![0.0, 0.0, 0.0, 0.0],
                        };
                        encoder_pub_cloned.publish(&joint_msg).ok();
                    }
                    Err(e) => eprintln!("Encoder read error: {}", e),
                }
                thread::sleep(Duration::from_secs(1));
            }
        });
    }

    println!("Node up. Subscribing on /cmd_vel, publishing BatteryState on /battery, and encoder counts on /encoder.");
    println!("Press Ctrl+C to exit. (Speeds=0 on exit.)");
    spin(node)?;
    Ok(())
}
