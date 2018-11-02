/**
 * @file      bb-blue-battery-state.cpp
 *
 * @brief     node to publish battery state for Beaglbone Blue
 *            based on the examples and librobotcontrol
 *						by James Strawson
 *
 * @license   MIT
 *
 * @author    usxbrix
 * @date      Nov 2018
 */


#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"

class BatteryState
{
protected:
  // Subscriber to thrusters use in percent
  

public:
  BatteryState(ros::NodeHandle &ros_node)
  {
    // init publishers
    battery_state_publisher_ = ros_node.advertise<sensor_msgs::BatteryState>("battery_state", 1);

    // battery present and full
    battery_msg_.percentage = 100;
    battery_msg_.present = 1;

  }

  void calculateBatteryCondition()
  {
     
    battery_msg_.percentage = 100;
 
  }

  void Publish()
  {
    battery_state_publisher_.publish(battery_msg_);
  }

protected:
  // publishers and messages to be published
  ros::Publisher battery_state_publisher_;
  sensor_msgs::BatteryState battery_msg_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "battery_state");
  ros::NodeHandle ros_node;

  ros::Rate loop_rate(1); // 1 hz

  BatteryState batteryState(ros_node);

  while (ros::ok())
  {
    batteryState.calculateBatteryCondition();
    batteryState.Publish();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
