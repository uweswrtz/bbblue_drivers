/*
 * Copyright (C) 2018, usxbrix
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>

#include <rc/motor.h>

// global variables
ros::Time g_msg_received;
bool g_driving = 0;

// %Tag(CALLBACK)%
void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  g_msg_received = ros::Time::now();
  ROS_INFO("cmd_vel Linear: [%f, %f, %f] Angular: [%f, %f, %f]", cmd_vel->linear.x, cmd_vel->linear.y, cmd_vel->linear.z, cmd_vel->angular.x, cmd_vel->angular.y, cmd_vel->angular.z);

  double dx = cmd_vel->linear.x;
  double dr = cmd_vel->angular.z;
  double dy = cmd_vel->linear.y;

  /*
  velocity_left_cmd = (linear_velocity – angular_velocity * WHEEL_BASE / 2.0)/WHEEL_RADIUS;
  velocity_right_cmd = (linear_velocity + angular_velocity * WHEEL_BASE / 2.0)/WHEEL_RADIUS;

  self.right = 1.0 * self.dx + self.dr * self.w / 2
  self.left = 1.0 * self.dx - self.dr * self.w / 2

  */

  double wb = 0.2; //wheel base
  double velocity_left = ( dx - dr * wb / 2.0);
  double velocity_right = ( dx + dr * wb / 2.0);

  ROS_INFO("set motor speed left: %f right: %f", velocity_left, velocity_right);

  // TODO DUTY = INPUT/MAX
  rc_motor_set(1,velocity_left);
  rc_motor_set(2,velocity_right);
  g_driving = 1;


}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{

  // ROS and node initialize
  ros::init(argc, argv, "diff_motor_driver");

  ros::NodeHandle n;

  g_msg_received = ros::Time::now();

  // get parameter

  int cmd_vel_timeout_;
  std::string base_frame_id_;

  n.param("cmd_vel_timeout", cmd_vel_timeout_, 10);
  n.param<std::string>("base_frame_id", base_frame_id_, "base_link");

  // initialize motor hardware first
  int pwm_freq_hz = RC_MOTOR_DEFAULT_PWM_FREQ; //25000
  if(rc_motor_init_freq(pwm_freq_hz))
  {
     ROS_ERROR("Initialize motor with [%d]: FAILED", pwm_freq_hz);
     return -1;
  }
  ROS_INFO("Initialize motor with [%d]: OK", pwm_freq_hz);


// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("cmd_vel", 100, cmd_velCallback);

// %EndTag(SUBSCRIBER)%

  ROS_INFO("Node is up and Subsciber started");

// %Tag(SPIN)%

  ros::Rate r(10);
  while (ros::ok())
  {
    ros::spinOnce();

    if ( g_driving && ( ros::Time::now().toSec() - g_msg_received.toSec() ) > cmd_vel_timeout_ )
    {
      ROS_INFO("TIMEOUT: No cmd_vel received: setting motors to 0");
      rc_motor_set(0,0);
      g_driving = 0;
    }
    r.sleep();
  }


// %EndTag(SPIN)%

// close motor hardware
  ROS_INFO("Calling rc_motor_cleanup()");
  rc_motor_cleanup();
  return 0;
}
// %EndTag(FULLTEXT)%
