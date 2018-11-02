/**
 * @file      bb-blue-battery-state.cpp
 *
 * @brief     node to publish battery state for Beaglbone Blue
 *            based on the examples and librobotcontrol
 *						by James Strawson
 *
 * @author    usxbrix
 * @date      Nov 2018
 *
 * @license
 * MIT License
 *
 * Copyright (c) 2018 usxbrix
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
