#pragma once

#include <pure_pursuit/pure_pursuit.h>

// 测试用日志输出宏
#define rosinfo ROS_INFO

// 直行/转弯开环测试类（独立于 PurePursuit，不改动 pure_pursuit.h）
class TestPursuit
{
public:
	TestPursuit(const ros::NodeHandle &nh);

	void initForROS();	// 初始化测试用的订阅/发布/服务
	void run();			// 测试主循环: 直行10s -> 转弯10s -> 结束

private:
	void callbackFromPos(const util::PositionConstPtr &msg_carposition);	// 小车位置回调
	void straight_publish();	// 发布直行指令
	void turn_publish();		// 发布转弯指令

	ros::NodeHandle nh_;
	ros::Subscriber sub_position_;			// 小车位置订阅者
	ros::Publisher pub_command_;			// 控制指令发布者

	util::Position car_position_;	// 小车实时位置
	ros::Time test_time_start_;		// 当前测试阶段开始时间
	int turn_test_flag_;			// 测试阶段标志: 0-直行 1-转弯 >=2-结束
	int test_flag_;					// 测试进行标志
};
