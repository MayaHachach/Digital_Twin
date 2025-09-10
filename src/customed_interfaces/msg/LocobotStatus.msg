string name

#joint_states
sensor_msgs/JointState joint_states

#from irobot_create_msgs/msg/WheelVels.msg
float32 velocity_left               # Velocity measure for left wheel in rad/sec.
float32 velocity_right              # Velocity measure for right wheel in rad/sec.
float32 linear_velocity             # Linear velocity measure in m/s.
float32 angular_velocity            # Angular velocity measure in rad/sec.

#from sensor_msgs/msg/BatteryState.msg
float32 voltage          # Voltage in Volts (Mandatory)
float32 current          # Negative when discharging (A)  (If unmeasured NaN)
float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
float32 temperature      # Temperature in Degrees Celsius
