#include "geometry.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <pure_pursuit/pure_pursuit.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <mower_msgs/ControlOk.h>
#include <yaml-cpp/yaml.h>

// ---------- 测试用全局变量声明 ----------
ros::Subscriber sub;              // 小车位置订阅者
ros::Publisher pub_command_;       // 控制指令发布者
util::Position car_position_;      // 小车实时位置
ros::Time test_time_start;        // 当前测试阶段开始时间
int turn_test_flag = 0;           // 测试阶段标志: 0-直行 1-转弯 >=2-结束
int test_flag = 0;                // 测试进行标志

void PurePursuit::initForROS()
{
    sub = nh_.subscribe("/Mower/position", 5, &PurePursuit::callbackFromPos, this);
    // setup publisher
    pub_command_ = nh_.advertise<mower_msgs::VehicleCmd>("/vehicle/cmd", 1);
}

void PurePursuit::callbackFromPos(const util::PositionConstPtr &msg_carposition) {
  car_position_ = *msg_carposition;
  last_pos_time = ros::Time::now();
}

void straight_publish()
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

void turn_publish()
{
    mower_msgs::VehicleCmd cmd_msg;
    cmd_msg.turn_value = 620;
    cmd_msg.drive_value = 0;
    cmd_msg.ad_control_enable = 1;
    cmd_msg.header.stamp = ros::Time::now();

    cmd_msg.mower_height = 0;
    cmd_msg.mover_bool = 0;
    cmd_msg.gear_model = cmd_msg.N_Gear;
    pub_command_.publish(cmd_msg);
}

void park_publish()
{
    mower_msgs::VehicleCmd cmd_msg;
    cmd_msg.turn_value = 0;
    cmd_msg.drive_value = 0;
    cmd_msg.ad_control_enable = 1;
    cmd_msg.header.stamp = ros::Time::now();

    cmd_msg.mower_height = 0;
    cmd_msg.mover_bool = 0;
    cmd_msg.gear_model = cmd_msg.p_Gear;
    pub_command_.publish(cmd_msg);
}

void PurePursuit::run()
{
    ros::Rate loop_rate(20);

    test_time_start = ros::Time::now();
    turn_test_flag = 0;
    test_flag = 1;
    rosinfo("car_position_x: %f , car_position_y: %f , car_position_yaw: %f", car_position_.position_x, car_position_.position_y, car_position_.yaw);

    while (ros::ok())
    {
        ros::spinOnce();

        if (test_flag == 1 && (ros::Time::now() - test_time_start).toSec() >= 10.0)
        {
            turn_test_flag++;
            rosinfo("usetime: %f", (ros::Time::now() - test_time_start).toSec());
            rosinfo("car_position_x: %f , car_position_y: %f , car_position_yaw: %f", car_position_.position_x, car_position_.position_y, car_position_.yaw);
            park_publish();
            test_time_start = ros::Time::now();
        }
        if(turn_test_flag == 0)
        {
            straight_publish();
        }
        else if(turn_test_flag == 1)
        {
            turn_publish();
        }else
        {
            test_flag = 0;
            rosinfo("test finish");
        }
        loop_rate.sleep()
    }
}