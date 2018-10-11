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

ros::Time msg_received = ros::Time::now();

// %Tag(CALLBACK)%
void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  msg_received = ros::Time::now();
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

  rc_motor_set(1,velocity_left);
  rc_motor_set(2,velocity_right);


  /*
  if cmd_vel->linear.x >0:
        ROS_INFO("Driving forward");
    //    forward(cmd_vel->linear.x)
    //   rc_motor_set(m1,cmd_vel->linear.x);
    //   rc_motor_set(m2,cmd_vel->linear.x);



    elif cmd_vel->linear.x <0:
        ROS_INFO("Driving backward");
      //  backward(abs(cmd_vel->linear.x))
      //   rc_motor_set(m1,cmd_vel->linear.x);
      //   rc_motor_set(m2,cmd_vel->linear.x);

    elif cmd_vel->angular.z > 0:
        ROS_INFO("Turning left");
      //  left(cmd_vel->angular.z)
      //   rc_motor_set(m1,cmd_vel->linear.x);
      //   rc_motor_set(m2,cmd_vel->linear.x);

    elif cmd_vel->angular.z < 0:
        ROS_INFO("Turning right");
      //  right(abs(cmd_vel->angular.z))
      //   rc_motor_set(m1,cmd_vel->linear.x);
      //   rc_motor_set(m2,cmd_vel->linear.x);


    else:
        ROS_INFO("Stopping");
      //  stop()
  */

}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "motor_listener_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // initialize hardware first
  int pwm_freq_hz = RC_MOTOR_DEFAULT_PWM_FREQ; //25000
  if(rc_motor_init_freq(freq_hz))
  {
     ROS_ERROR("Initialize motor with [%d]: FAILED");
     return -1;
  }
  ROS_INFO("Initialize motor with [%d]: OK");
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("cmd_vel", 100, cmd_velCallback);

// %EndTag(SUBSCRIBER)%
  ROS_INFO("Node is up and Subsciber started");
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  //ros::spin();
  ros::Rate r(10);
  while (ros::ok())
  {
    ros::spinOnce();

    if ( ( ros::Time::now().toSec() - msg_received.toSec() ) > 0.25 )
    {
      ROS_INFO("No cmd_vel received: setting motors to 0");
      rc_motor_set(0,0);
    }
    r.sleep();
  }


// %EndTag(SPIN)%

  ROS_INFO("Calling rc_motor_cleanup()");
  rc_motor_cleanup();
  return 0;
}
// %EndTag(FULLTEXT)%
