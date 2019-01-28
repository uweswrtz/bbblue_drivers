/**
 * @file bb-blue-imu-mpu9250-node.cpp
 *
 *
 * @brief     node to publish IMU data from MPU9250 on Beaglbone Blue
 *            based on the rc_test_dmp.c example from librobotcontrol
 *						by James Strawson
 *
 * @author    usxbrix
 * @date      Oct 2018
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


#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h> // for atoi() and exit()
#include <rc/mpu.h>
#include <rc/time.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/MagneticField.h"

// bus for Robotics Cape and BeagleboneBlue is 2, interrupt pin is on gpio3.21
// change these for your platform
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

// Global Variables
static int running = 0;
static int silent_mode = 0;
static int show_accel = 1;
static int show_gyro  = 1;
static int enable_mag = 1;
static int show_compass = 1;
static int show_temp  = 0;
static int show_quat  = 1;
static int show_tb = 0;
static int orientation_menu = 0;
static int sample_rate = 4; //sample_rate must be between 4 & 200
static int priority;
static rc_mpu_data_t data;

// local functions

static void __print_data(void);
static void __pub_data(void);


ros::Publisher imu_pub;
ros::Publisher mag_pub;
std::string imu_frame_id_;




/**
 * This is the IMU interrupt function to print data.
 */
static void __print_data(void)
{
	ROS_INFO("Comp[raw]:%6.1f[%6.1f] Quat:%4.1f %4.1f %4.1f %4.1f Accel:%5.2f %5.2f %5.2f Gyro:%5.1f %5.1f %5.1f ", \
	data.compass_heading*RAD_TO_DEG, \
	data.compass_heading_raw*RAD_TO_DEG, \
	data.fused_quat[QUAT_W], \
	data.fused_quat[QUAT_X], \
	data.fused_quat[QUAT_Y], \
	data.fused_quat[QUAT_Z], \
	data.accel[0],\
	data.accel[1],\
	data.accel[2],\
	data.gyro[0],\
	data.gyro[1],\
	data.gyro[2]);
	return;
}

/**
 * This is the IMU interrupt function to publish data.
 */
static void __pub_data(void)
{

	sensor_msgs::Imu imu_msg;

	ros::Time current_time = ros::Time::now();

	imu_msg.header.stamp = current_time;
	imu_msg.header.frame_id = imu_frame_id_;

// test unfused data
	imu_msg.orientation.x = data.dmp_quat[QUAT_X];
  	imu_msg.orientation.y = data.dmp_quat[QUAT_Y];
	imu_msg.orientation.z = data.dmp_quat[QUAT_Z];
  	imu_msg.orientation.w = data.dmp_quat[QUAT_W];

/* 	imu_msg.orientation.x = data.fused_quat[QUAT_X];
  imu_msg.orientation.y = data.fused_quat[QUAT_Y];
	imu_msg.orientation.z = data.fused_quat[QUAT_Z];
  imu_msg.orientation.w = data.fused_quat[QUAT_W]; */

  imu_msg.angular_velocity.x = data.gyro[0]*3.14159265358979323846/180;
  imu_msg.angular_velocity.y = data.gyro[1]*3.14159265358979323846/180;
  imu_msg.angular_velocity.z = data.gyro[2]*3.14159265358979323846/180;

  imu_msg.linear_acceleration.x = data.accel[0];
  imu_msg.linear_acceleration.y = data.accel[1];
  imu_msg.linear_acceleration.z = data.accel[2];

	/**
	 * covariances used from razor_imu_9dof
	 * https://github.com/KristofRobot/razor_imu_9dof/blob/indigo-devel/nodes/imu_node.py
	 */
	imu_msg.orientation_covariance[0] = 0.0025 ;
	imu_msg.orientation_covariance[1] = 0;
	imu_msg.orientation_covariance[2] = 0;
	imu_msg.orientation_covariance[3] = 0;
	imu_msg.orientation_covariance[4] = 0.0025 ;
	imu_msg.orientation_covariance[5] = 0;
	imu_msg.orientation_covariance[6] = 0;
	imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.0025 ;

	imu_msg.angular_velocity_covariance [0] = 0.02 ;
	imu_msg.angular_velocity_covariance [1] = 0;
	imu_msg.angular_velocity_covariance [2] = 0;
	imu_msg.angular_velocity_covariance [3] = 0;
	imu_msg.angular_velocity_covariance [4] = 0.02 ;
	imu_msg.angular_velocity_covariance [5] = 0;
	imu_msg.angular_velocity_covariance [6] = 0;
	imu_msg.angular_velocity_covariance [7] = 0;
	imu_msg.angular_velocity_covariance [8] = 0.02 ;

	imu_msg.linear_acceleration_covariance[0] = 0.04 ;
	imu_msg.linear_acceleration_covariance[1] = 0;
	imu_msg.linear_acceleration_covariance[2] = 0;
	imu_msg.linear_acceleration_covariance[3] = 0;
	imu_msg.linear_acceleration_covariance[4] = 0.04 ;
	imu_msg.linear_acceleration_covariance[5] = 0;
	imu_msg.linear_acceleration_covariance[6] = 0;
	imu_msg.linear_acceleration_covariance[7] = 0;
	imu_msg.linear_acceleration_covariance[8] = 0.04 ;


	imu_pub.publish(imu_msg);

	sensor_msgs::MagneticField mag_msg;
  mag_msg.header = imu_msg.header;

	mag_msg.magnetic_field.x = data.mag[0]/1000000;
	mag_msg.magnetic_field.y = data.mag[1]/1000000;
	mag_msg.magnetic_field.z = data.mag[2]/1000000;

	// TODO: covariance for magnetic field

  mag_pub.publish(mag_msg);

	return;

}


/**
 * @brief      interrupt handler to catch ctrl-c
 */
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}


/**
 * main() serves to parse user options, initialize the imu and interrupt
 * handler, and wait for the rc_get_state()==EXITING condition before exiting
 * cleanly. The imu_interrupt function print_data() is what actually prints new
 * imu data to the screen after being set with rc_mpu_set_dmp_callback().
 *
 * @param[in]  argc  The argc
 * @param      argv  The argv
 *
 * @return     0 on success -1 on failure
 */
int main(int argc, char *argv[])
{
	int show_something = 0; // set to 1 when any show data option is given.

	// start with default config and modify based on options
	rc_mpu_config_t conf = rc_mpu_default_config();
	conf.i2c_bus = I2C_BUS;
	conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
	conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;

	conf.enable_magnetometer = enable_mag;
	conf.dmp_fetch_accel_gyro=1;
	conf.dmp_sample_rate = sample_rate;

 	// priority option
	//conf.dmp_interrupt_priority = priority;
	//conf.dmp_interrupt_sched_policy = SCHED_FIFO;

	// magnetometer option
	//	conf.read_mag_after_callback = 0;

	// print warnings
	// conf.show_warnings=1;

	/*
	// If the user gave the -o option to select an orientation then prompt them
	if(orientation_menu){
		conf.orient=__orientation_prompt();
	}

	*/

	ros::init(argc, argv, "imu_pub_node");
	ros::NodeHandle n("imu");
	n.param<std::string>("frame_id", imu_frame_id_, "imu_link");
	ROS_INFO("FrameID: %s",imu_frame_id_.c_str());

	//ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_publisher", 10);
	imu_pub = n.advertise<sensor_msgs::Imu>("data", 1, false);
	//imu_pub = n.advertise<sensor_msgs::Imu>("data_raw", 1, false);
	mag_pub = n.advertise<sensor_msgs::MagneticField>("mag", 1, false);

	ros::Rate loop_rate(10);

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running = 1;

	// now set up the imu for dmp interrupt operation
	if(rc_mpu_initialize_dmp(&data, conf)){
		ROS_INFO("rc_mpu_initialize_failed");
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}
	ROS_INFO("MPU initialized");

	//if(!silent_mode) rc_mpu_set_dmp_callback(&__print_data);
	if(!silent_mode) rc_mpu_set_dmp_callback(&__pub_data);
	//now just wait, print_data() will be called by the interrupt
	while(running)	rc_usleep(100000);

	// shut things down
	rc_mpu_power_off();
	printf("\n");
	fflush(stdout);
	return 0;
}
