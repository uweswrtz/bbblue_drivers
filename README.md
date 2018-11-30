# bbblue_drivers

some ROS drivers for the Beaglbone Blue

>WIP!!! Ugly code included!!!

## Nodes overview

* differential driver node using the onboard motor ports
* publisher for IMU messages from MPU9250
* publisher for battery state


## Differential Motor Driver

### Parameter

* ~left_motor = 1
* ~right_motor = 2
* ~timeout = 5
* ~maxspeed = 0.4
* ~minspeed = 0.1
* ~wheelbase = 0.2
* ~turnspeed = 1
* ~duty_factor = 1  - velocity to PWM duty cycle factor (simple linear approach)

### Run

`rosrun ros-blue diff_motor_driver`

`rosrun ros-blue diff_motor_driver _left_motor:=3 _right_motor:=4 _minspeed:=0.137 _maxspeed:=0.364 _duty_factor:=2.2`

Publish to cmd_vel manually

`rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0]'``

## Batter State Node

### Run

`rosrun ros-blue battery_state`



## IMU node

Start the IMU node

`rosrun ros-blue imu_pub_node`

### Visualizing IMU with rviz

A static transformation is required:

`rosrun tf static_transform_publisher 0.0 0.0 0.0 0 0 0 map imu_link 10`

The Beaglebone Blue has no display port. So for visualization an aditional system is required.

`export ROS_MASTER_URI=http://rosbot:11311`

Starting rviz after exporting the MASTER usxbrix

`rviz`

red - x green - y blue -z

### optional IMU tools

`sudo apt-get install ros-melodic-imu-tools`

`rosrun imu_filter_madgwick imu_filter_node`
