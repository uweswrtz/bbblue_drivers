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
#include <rc/adc.h>
//#include <stdlib.h> // for atoi() and exit()

#define VOLTAGE_DISCONNECT      1 // Threshold for detecting disconnected battery

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
    //battery_msg_.percentage = 100;
    //battery_msg_.present = 1;
    battery_msg_.power_supply_technology = 3;

  }

  void calculateBatteryCondition()
  {
    double pack_voltage;    // 2S pack voltage on JST XH 2S balance connector
    double cell_voltage;    // cell voltage
    double jack_voltage;    // could be dc power supply or another battery



    // read in the voltage of the 2S pack and DC jack
    pack_voltage = rc_adc_batt();
    jack_voltage = rc_adc_dc_jack();
    // sanity check the SDC didn't return an error
    if(pack_voltage<0.0 || jack_voltage<0.0){
      ROS_ERROR("ERROR: can't read voltages");
      //fprintf(stderr,"ERROR: can't read voltages\n");
      return -1;
    }
    // check if a pack is on the 2S balance connector
    if(pack_voltage<VOLTAGE_DISCONNECT){
      pack_voltage = 0;
      battery_msg_.present = 0;
    }
    if(jack_voltage<VOLTAGE_DISCONNECT){
      jack_voltage = 0;
    }
    // 2S pack, so divide by two for cell voltage
    cell_voltage = pack_voltage/2;

    ROS_INFO("Pack: %0.2lfV   Cell: %0.2lfV   DC Jack: %0.2lfV  ", \
      pack_voltage, cell_voltage, jack_voltage);

    //battery_msg_.percentage = 100;
    battery_msg_.voltage = pack_voltage;

    /** TODO: percentage
    * https://github.com/StrawsonDesign/librobotcontrol/blob/master/services/rc_battery_monitor/src/rc_battery_monitor.c
    *
    * battery_msg_.percentage = 100;
    */

    /** TODO: power_supply_status,power_supply_health
    *
    */
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

  ROS_INFO("Initializing node %s in namespace: %s", ros::this_node::getName().c_str(), ros::this_node::getNamespace().c_str() );

  // TODO: use parameters
  int pst; //power_supply_technology
  ros_node.param("~power_supply_technology", pst, 3);

  ros::Rate loop_rate(1); // 1 hz

  if(rc_adc_init()==-1)
  {
     ROS_ERROR("Initialize ADC: FAILED");
     return -1;
  }

  BatteryState batteryState(ros_node);

  while (ros::ok())
  {
    batteryState.calculateBatteryCondition();
    batteryState.Publish();

    ros::spinOnce();
    loop_rate.sleep();
  }

  rc_adc_cleanup();
  return 0;
}
