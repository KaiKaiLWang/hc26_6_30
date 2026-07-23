#include "geometry.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <pure_pursuit/pure_pursuit.h>
#include "pure_pursuit/test_pursuit.h"
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <mower_msgs/ControlOk.h>
#include <yaml-cpp/yaml.h>

TestPursuit::TestPursuit(const ros::NodeHandle &nh) : nh_(nh)
{
	turn_test_flag_ = 0;
	test_flag_ = 0;

	initForROS();
	run();
}

void TestPursuit::initForROS()
{
	sub_position_ = nh_.subscribe("/Mower/position", 5, &TestPursuit::callbackFromPos, this);
	// setup publisher
	pub_command_ = nh_.advertise<mower_msgs::VehicleCmd>("/vehicle/cmd", 1);
}

void TestPursuit::callbackFromPos(const util::PositionConstPtr &msg_carposition)
{
	car_position_ = *msg_carposition;
}

void TestPursuit::straight_publish()
{
	mower_msgs::VehicleCmd cmd_msg;
	cmd_msg.turn_value = 0;
	cmd_msg.drive_value = 1;
	cmd_msg.ad_control_enable = 1;
	cmd_msg.header.stamp = ros::Time::now();

	cmd_msg.mower_height = 0;
	cmd_msg.mover_bool = 0;
	cmd_msg.gear_model = cmd_msg.D_Gear;
	pub_command_.publish(cmd_msg);
}

void TestPursuit::turn_publish()
{
	mower_msgs::VehicleCmd cmd_msg;
	cmd_msg.turn_value = 62;
	cmd_msg.drive_value = 0;
	cmd_msg.ad_control_enable = 1;
	cmd_msg.header.stamp = ros::Time::now();

	cmd_msg.mower_height = 0;
	cmd_msg.mover_bool = 0;
	cmd_msg.gear_model = cmd_msg.N_Gear;
	pub_command_.publish(cmd_msg);
}

void TestPursuit::run()
{
	ros::Rate loop_rate(20);

	test_time_start_ = ros::Time::now();
	turn_test_flag_ = 0;
	test_flag_ = 1;
	rosinfo("car_position_x: %f , car_position_y: %f , car_position_yaw: %f", car_position_.position_x, car_position_.position_y, car_position_.yaw);

	while (ros::ok())
	{
		ros::spinOnce();

		if (test_flag_ == 1 && (ros::Time::now() - test_time_start_).toSec() >= 10.0)
		{
			turn_test_flag_++;
			rosinfo("usetime: %f", (ros::Time::now() - test_time_start_).toSec());
			rosinfo("car_position_x: %f , car_position_y: %f , car_position_yaw: %f", car_position_.position_x, car_position_.position_y, car_position_.yaw);
			test_time_start_ = ros::Time::now();
		}
		if(turn_test_flag_ == 0)
		{
			straight_publish();
		}
		else if(turn_test_flag_ == 1)
		{
			turn_publish();
		}else
		{
			test_flag_ = 0;
			rosinfo("test finish");
		}
	}
}
