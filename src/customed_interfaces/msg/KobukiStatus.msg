string name

#from irobot_create_msgs/msg/WheelVels.msg
float32 velocity_left               # Velocity measure for left wheel in rad/sec.
float32 velocity_right              # Velocity measure for right wheel in rad/sec.
float32 linear_velocity             # Linear velocity measure in m/s.
float32 angular_velocity            # Angular velocity measure in rad/sec.

#from irobot_create_msgs/msg/WheelStatus.msg
int16 current_ma_right              # Current measurement for right wheel in milliamps
int16 current_ma_left               # Current measurement for left wheel in milliamps 
int16 pwm_left                      # PWM % duty cycle measurement (where int16::max is +100%) for left wheel 
int16 pwm_right                     # PWM % duty cycle measurement (where int16::max is +100%) for right wheel
bool wheels_enabled                 # Whether wheels are enabled or disabled (disabled when E-Stopped)

#from sensor_msgs/msg/BatteryState.msg
float32 voltage          # Voltage in Volts (Mandatory)
float32 temperature      # Temperature in Degrees Celsius (If unmeasured NaN)
float32 current          # Negative when discharging (A)  (If unmeasured NaN)
float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)