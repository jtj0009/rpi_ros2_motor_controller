## To Test
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" 

ros2 topic echo /battery sensor_msgs/msg/BatteryState
