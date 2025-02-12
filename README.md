#About
This Rust application controls a two-motor drive system on a Raspberry Pi using I²C (with an encoder/motor driver at 0x34) and ROS 2 messages. It:

Subscribes to geometry_msgs/msg/Twist on the topic /cmd_vel to set left/right track speeds.
Publishes battery status on the topic /battery as sensor_msgs/msg/BatteryState.
Manages I²C hardware initialization and sets motor speeds to zero on shutdown.
Features

## I²C Motor Control:
Opens /dev/i2c-<BUS> and configures the motor driver at address 0x34.
Accepts speed commands in range [-100..100] for each motor (M1, M2).
ROS 2 Integration:
Subscriber on /cmd_vel (type Twist):
Interprets linear.x and angular.z to compute track speeds, then writes them via I²C.
Publisher on /battery:
Periodically reads the battery voltage (in mV) from the I²C device at 0x00.
Publishes a fully populated BatteryState message (voltage, design capacity = 6.0 Ah, discharge status, etc.).
Reconnect Loop:
If the board is powered off at boot, it retries every 3 s until the I²C driver is responsive.
Auto Zero on Exit:
Uses a Drop implementation to set both motors to speed=0 if the process ends or is interrupted.
Prerequisites

## Hardware:
Raspberry Pi (Pi 4 or Pi 5).
I²C motor driver at address 0x34 (e.g. TT motor, JGB37, etc.).
Battery voltage read at register ADC_BAT_ADDR = 0x00.
I²C Enabled:
In /boot/config.txt (or /boot/firmware/config.txt on Ubuntu Pi), set dtparam=i2c_arm=on.
Reboot, confirm the device with i2cdetect -y 1 (or appropriate bus).
ROS 2 installed**:
This code uses rclrs, so you need a functioning ROS 2 environment and the ros2-rust pipeline or a suitable setup with rclrs available.
Rust with cargo**:
rustc 1.65+ recommended.
rppal, rclrs, geometry_msgs, sensor_msgs crates properly patched or generated locally if using custom messages.
##Building & Running

Clone the repository (or place the .rs file in your workspace).
Set up your ROS 2 + rclrs environment. For example:
source /opt/ros/humble/setup.bash
Build:
cargo build --release
Run:
./target/release/motor_controller
The app will keep retrying if it can’t open the I²C bus.
On success, it will spin up a ROS 2 node named tank_control_node, subscribe to /cmd_vel, and publish to /battery.
## Usage Notes

Controlling Speeds:
Publish Twist messages on /cmd_vel, e.g.:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.0}}'
This might yield a left+right speed ~30 depending on scaling.
Battery:
The code reads the voltage in millivolts (2-byte register).
Publishes BatteryState messages on /battery.
Nominally every 5 seconds.
Coordinate System:
By default, linear.x controls forward/back.
angular.z controls left/right turning.
The function twist_to_tracks() does a simple differential drive calculation with linear_scale = 60.0 and angular_scale = 40.0, then clamps to [-100..100].
Shutdown Behavior:
If you press Ctrl+C, the node stops spinning and the MotorDriver's Drop sets speeds to 0.
Great for safety so the motors don’t keep spinning.
Systemd Integration

## To run at boot:

move systemd service file into /etc/system/systemd/, systemctl daemon-reload, systemctl enable, and systemctl restart

## To Test
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" 

ros2 topic echo /battery sensor_msgs/msg/BatteryState

## Troubleshooting

No I²C device found:
Check i2cdetect -y 1 (or -y 0 / -y 10 on Pi 5 if enumerated differently). Confirm 0x34 is listed.
Ensure overlays in config.txt are correct.
Battery read errors:
If the driver doesn’t actually store voltage at 0x00, you may see repeated errors or 0 voltage.
Modify ADC_BAT_ADDR accordingly.
No cmd_vel response:
Confirm you’re actually publishing on /cmd_vel with ros2 topic echo /cmd_vel.
Ensure the node started without error and shows Node up. Subscribing on /cmd_vel... in logs.
