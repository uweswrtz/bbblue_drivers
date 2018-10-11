# ros-blue

some ROS nodes for the Beaglbone Blue

## nodes

* differential drive controller node using the onboard motor ports
* publisher for IMU messages from MPU9250


 sudo apt-get install ros-melodic-imu-tools 


rosrun imu_filter_madgwick imu_filter_node
rosrun tf static_transform_publisher 0.0 0.0 0.0 0 0 0 map imu_link 10

red - x green - y blue -z 
