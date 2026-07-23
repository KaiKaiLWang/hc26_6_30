#include "pure_pursuit/pure_pursuit.h"
#include "geometry.h"

PurePursuit::PurePursuit(const ros::NodeHandle &nh) : nh_(nh)
{
	running_state_.store(RunStateValue::Follow);
	low_speed_flag_.store(false);
	stop_car_.store(false);
	avoid_state_ = 0;
	last_local_path_.point_num = 0;

	mover_bool_cfg_ = 0;
	mower_height_cfg_ = 0;

	nh.param<float>("pure_pursuit/v_min", v_min_, 0.2f);
	nh.param<float>("pure_pursuit/w_max", w_max_, 0.15f);
	nh.param<float>("pure_pursuit/min_lookahead_distance", min_lookahead_distance_, 1.5f);
	nh.param<float>("pure_pursuit/regulated_min_radius", regulated_min_radius_, 2.f);
	nh.param<float>("pure_pursuit/low_speed", low_speed_, 0.3f);
	// ROS_INFO("v_min: %f, w_max: %f, min_lookahead_distance: %f, regulated_min_radius: %f", v_min_, w_max_, min_lookahead_distance_, regulated_min_radius_);

	sub_position_ = nh_.subscribe("/Mower/position", 1, &PurePursuit::positionCallback, this);
	sub_local_path_ = nh_.subscribe("/lawn_mower/global_path", 1, &PurePursuit::localPathCallback, this);
	sub_stop_signal_ = nh_.subscribe("/mower/stop_car", 1, &PurePursuit::stopSignalCallback, this);
	sub_avoid_state_ = nh_.subscribe("/lawn_mower/avoid_state", 1, &PurePursuit::avoidstateCallback, this);
	sub_signal_ = nh_.subscribe("/signal", 1, &PurePursuit::signalCallback, this);

	pub_command_ = nh_.advertise<mower_msgs::VehicleCmd>("/vehicle/cmd", 1);

	turn_completed_srv_ = nh_.advertiseService("/turn_completed", &PurePursuit::turnCompletedCallback, this);

	ROS_INFO("Pure Pursuit node started");

	state_machine_run();
}

void PurePursuit::positionCallback(const util::PositionConstPtr &msg_carposition)
{

	std::lock_guard<std::mutex> lock_local_waypoints(local_waypoints_mutex_);
	
	util::LocalPath last_local_path;
	{
		std::lock_guard<std::mutex> lock_local_path(local_path_mutex_);
		last_local_path = last_local_path_;
	}
	car_position_ = *msg_carposition;
	last_pos_time_ = ros::Time::now();

	float theta = car_position_.yaw;
	float cos_val = cosf(theta);
	float sin_val = sinf(theta);
	local_waypoints_.clear();
	local_waypoints_.reserve(last_local_path.point_num);
	for (int i = 0; i < last_local_path.point_num; i++)
	{
		// ROS_INFO("索引：%d, 路径点(%.3f, %.3f, %.3f), gear: %d", i, last_local_path.x[i], last_local_path.y[i], last_local_path.heading[i], last_local_path.gear[i]);
		waypoint waypoint;
		waypoint.global_x = last_local_path.x[i];
		waypoint.global_y = last_local_path.y[i];
		waypoint.global_yaw = last_local_path.heading[i];
		double dx = last_local_path.x[i] - car_position_.position_x;
		double dy = last_local_path.y[i] - car_position_.position_y;
		// 转换到车辆坐标系
		waypoint.local_x = dx * cos_val + dy * sin_val;
		waypoint.local_y = dx * (-sin_val) + dy * cos_val;
		waypoint.local_yaw = robot::geometry::NormalizeAngle(last_local_path.heading[i] - theta);
		waypoint.gear = last_local_path.gear[i];
		waypoint.speed = last_local_path.speed[i];
		local_waypoints_.push_back(waypoint);
	}
}

void PurePursuit::localPathCallback(const util::LocalPathConstPtr &msg_planning_result)
{
	std::lock_guard<std::mutex> lock(local_path_mutex_);
	last_local_path_time_ = ros::Time::now();
	if (last_local_path_ != *msg_planning_result)
	{
		last_local_path_ = *msg_planning_result;
		ROS_INFO("五维路径更新");
	}
	else
	{
		ROS_INFO("五维路径未更新");
	}
}

void PurePursuit::stopSignalCallback(const std_msgs::BoolConstPtr &msg_stop_signal)
{
	if (msg_stop_signal->data)
	{
		stop_car_.store(true);
		ROS_INFO("收到停车信号");
	}
	else
	{
		stop_car_.store(false);
		ROS_INFO("收到继续行驶信号");
	}
}
void PurePursuit::avoidstateCallback(const std_msgs::UInt8ConstPtr &avoid_msg)
{
	std::lock_guard<std::mutex> lock(avoid_state_mutex_);
	avoid_state_ = avoid_msg->data;
}

void PurePursuit::signalCallback(const std_msgs::StringConstPtr &msg_signal)
{
	std::lock_guard<std::mutex> lock(mover_mutex_);
	if (msg_signal->data == "rise") {
		mower_height_cfg_ = 1;
		ROS_INFO("mower_height = 1");
	}
	if (msg_signal->data == "falling") {
		mower_height_cfg_ = 2;
		ROS_INFO("mower_height = 2");
	}
	if (msg_signal->data == "open") {
		mover_bool_cfg_ = 1;
		ROS_INFO("mover_bool_cfg = 1");
	}
	if (msg_signal->data == "close") {
		mover_bool_cfg_ = 0;
		ROS_INFO("mover_bool_cfg = 0");
	}
	if (msg_signal->data == "stop" || msg_signal->data == "mowing_finished" || msg_signal->data == "reset")
	{
		std::lock_guard<std::mutex> lock1(avoid_state_mutex_);
		std::lock_guard<std::mutex> lock2(local_path_mutex_);
		std::lock_guard<std::mutex> lock3(mover_mutex_);
		std::lock_guard<std::mutex> lock4(turn_completed_mutex_);
		low_speed_flag_.store(false);
		stop_car_.store(false);
		avoid_state_ = 0;
		last_local_path_.point_num = 0;
		mover_bool_cfg_ = 0;
		mower_height_cfg_ = 0;
		turn_completed_points_.clear();
		ROS_INFO("Pure Pursuit node reset, msg is %s", msg_signal->data.c_str());
	}
}

bool PurePursuit::turnCompletedCallback(pure_pursuit::TurnCompleted::Request &req, pure_pursuit::TurnCompleted::Response &res)
{
	std::lock_guard<std::mutex> lock(turn_completed_mutex_);
	auto it = std::find_if(turn_completed_points_.begin(), turn_completed_points_.end(), [&](const waypoint& pt)
			{
				return fabs(req.x - pt.global_x) < 1e-4f &&
						fabs(req.y - pt.global_y) < 1e-4f;
			});

	if (it != turn_completed_points_.end())
	{
		ROS_INFO("point: (%f, %f)转弯完成, gear = %d，反馈成功",
				req.x, req.y, it->gear);
		res.turn_finish = true;
		return true;
	}
	else
	{
		res.turn_finish = false;
		return true;
	}
}

int PurePursuit::findNearestPoint(const std::vector<waypoint> &local_waypoints)
{
	float min_distance = std::numeric_limits<float>::max();
	int nearest_waypoint_index = -1;
	for (int i = 0; i < local_waypoints.size(); i++)
	{
		float distance = hypot(local_waypoints[i].local_x, local_waypoints[i].local_y);
		if (distance < min_distance)
		{
			min_distance = distance;
			nearest_waypoint_index = i;
		}
	}
	if (nearest_waypoint_index != -1)
		ROS_INFO("离车辆最近的点索引: %d, 距离: %.3f m", nearest_waypoint_index, min_distance);
	return nearest_waypoint_index;
}

waypoint PurePursuit::calculateLookaheadPoint(const std::vector<waypoint> &local_waypoints, const int &nearest_waypoint_index, const float &v_expect)
{
	waypoint lookahead_waypoint;
	lookahead_waypoint.gear = -1;
	float lookahead_distance = min_lookahead_distance_ + v_expect * 0.7f;
	int point_num = std::ceil(lookahead_distance / 0.25f);	// 中间点数量
	int sum_points_num = nearest_waypoint_index + point_num;
	if (sum_points_num >= local_waypoints.size())	// 防止越界
		sum_points_num = local_waypoints.size() - 1;

	lookahead_waypoint = local_waypoints[sum_points_num];
	for (int i = nearest_waypoint_index; i <= sum_points_num; i++)
	{
		if (local_waypoints[i].gear == 2)
		{
			// 如果中间有转向点，则转向点就是预瞄点
			lookahead_waypoint = local_waypoints[i];
			break;
		}
	}
	return lookahead_waypoint;
}

Twist PurePursuit::calculate_PurePursuit(const float &v_expect, const waypoint &lookahead_point)
{
	Twist cmd;
	float linear_vel = v_expect;
	// 前瞄距离的平方
	float ld2 = lookahead_point.local_x * lookahead_point.local_x + lookahead_point.local_y * lookahead_point.local_y;
	// 防止除零
	if (ld2 < 1e-4)
	{
		cmd.linear = 0.0;
		cmd.angular = 0.0;
		return cmd;
	}
	// 如果接近预瞄点则减速
	if (ld2 < 1.0 && ld2 > 0.5 * 0.5)
		linear_vel = std::min(linear_vel, 0.5f);
	if (ld2 < 0.5 * 0.5)
		linear_vel = std::min(linear_vel, 0.25f);

	// Pure Pursuit 曲率
	float curvature = 2.0 * lookahead_point.local_y / ld2;
	if (fabs(curvature) < 1e-4)
	{
		cmd.linear = linear_vel;
		cmd.angular = 0;
		return cmd;
	}

	const float radius = std::fabs(1.0 / curvature);
	ROS_INFO("转弯半径：%.3f m", radius);
	if (radius < regulated_min_radius_)
		cmd.linear = linear_vel * (radius / regulated_min_radius_);
	else
		cmd.linear =  linear_vel;

	cmd.linear = std::max(cmd.linear, static_cast<double>(v_min_)); // 最小速度限制
	if (cmd.linear * curvature >= 0)	
		cmd.angular = std::min(cmd.linear * curvature, static_cast<double>(w_max_));
	else
		cmd.angular = std::max(cmd.linear * curvature, static_cast<double>(-w_max_));

	return cmd;
}

void PurePursuit::publishCommand(const Twist &cmd)
{
	mower_msgs::VehicleCmd cmd_msg;
	cmd_msg.header.stamp = ros::Time::now();
	cmd_msg.drive_value = static_cast<int>(cmd.linear * 100.0f); // 转化成cm/s
	// 单片机端角速度方向是左手坐标系，并且需要取整[由于100会导致精度损失，改为乘以1000]
	cmd_msg.turn_value = static_cast<int>(cmd.angular * 2000.0f);//2000是为了可以在取整时对小数点后面四舍五入
	cmd_msg.turn_value = -cmd_msg.turn_value;
	cmd_msg.ad_control_enable = 1;
	if (fabs(cmd.linear) < 1e-4f && fabs(cmd.angular) < 1e-4f)
		cmd_msg.gear_model = cmd_msg.p_Gear;
	else
		cmd_msg.gear_model = cmd_msg.D_Gear;
	
	{
		std::lock_guard<std::mutex> lock(mover_mutex_);
		cmd_msg.mover_bool = mover_bool_cfg_;
		cmd_msg.mower_height = mower_height_cfg_;
	}
	if (stop_car_.load())
	{
		cmd_msg.drive_value = 0;
		cmd_msg.turn_value = 0;
		cmd_msg.gear_model = cmd_msg.p_Gear;
	}
	pub_command_.publish(cmd_msg);
	ROS_INFO("当前线速度: %.3f m/s, 当前角速度: %.3f rad/s",
		static_cast<double>(cmd_msg.drive_value) / 100.0,
		static_cast<double>(cmd_msg.turn_value) / 2000.0);
}

// 计算原地旋转的速度
float PurePursuit::calculate_rotate_vel(const float &yaw)
{
	ROS_INFO("角度差：%.3f", yaw);
	float cur_yaw = yaw;
	// 如果角度差在180度附近就取180度，防止正负左右横跳
	if (fabs(fabs(yaw) - M_PI) < 5.0 * M_PI / 180.0)
		cur_yaw = M_PI;
	float angular_vel;
	if (fabs(cur_yaw) > M_PI_2/2)
		angular_vel = cur_yaw > 0 ? 0.5 : -0.5;
	else if (fabs(cur_yaw) > M_PI_2/6)
		angular_vel = cur_yaw > 0 ? 0.4 : -0.4;
	else
		angular_vel = cur_yaw > 0 ? 0.3 : -0.3;
	return angular_vel;
}

void PurePursuit::state_machine_run()
{
	ros::Rate loop_rate(20);
	while (ros::ok())
	{
		ros::spinOnce();
		Twist twist_cmd;
		int nearest_waypoint_index = -1;	// 离车辆最近的点索引
		std::vector<waypoint> local_waypoints; // 在小车坐标系下的五维路径点
		float v_expect = 0;	// 期望速度
		low_speed_flag_.store(false);
		if (ros::Time::now() - last_pos_time_ > ros::Duration(1.0))
		{
			ROS_WARN("车辆位置信息丢失");
			running_state_.store(RunStateValue::Stop);
			goto STATEMACHINE;
		}
		if (ros::Time::now() - last_local_path_time_ > ros::Duration(3.0))
		{
			ROS_WARN("五维路径信息丢失");
			running_state_.store(RunStateValue::Stop);
			goto STATEMACHINE;
		}
		{
			std::lock_guard<std::mutex> lock(avoid_state_mutex_);
			if (avoid_state_ == 4)
			{
				ROS_WARN("紧急预警，需倒车");
				running_state_.store(RunStateValue::Reverse);
				goto STATEMACHINE;
			}
			if (avoid_state_ == 3)
			{
				ROS_WARN("紧急预警，需停车");
				running_state_.store(RunStateValue::Stop);
				goto STATEMACHINE;
			}
			if (avoid_state_ == 2)
			{
				ROS_WARN("有较远障碍物，普通预警，需低速");
				low_speed_flag_.store(true);
			}
		}

		{
			std::lock_guard<std::mutex> lock_local_waypoints(local_waypoints_mutex_);
			local_waypoints = local_waypoints_;
		}

		if (running_state_.load() == RunStateValue::Turn)
		{
			ROS_INFO("小车正在转弯，等待转弯完成");
			// 更新预瞄点角度
			bool renew_lookahead_yaw = false;
			for (uint i = 0; i < local_waypoints.size(); i++)
			{
				ROS_INFO("路径点： x = %.3f, y = %.3f, gear = %d", local_waypoints[i].global_x, local_waypoints[i].global_y, local_waypoints[i].gear);
				if (lookahead_waypoint_.global_x == local_waypoints[i].global_x && 
					lookahead_waypoint_.global_y == local_waypoints[i].global_y && 
					local_waypoints[i].gear == 2)
				{
					lookahead_waypoint_.local_yaw = local_waypoints[i].local_yaw;
					renew_lookahead_yaw = true;
					ROS_INFO("预瞄点角度已更新");
					break;
				}
			}
			if (!renew_lookahead_yaw)
				ROS_WARN("预瞄点角度未更新，可能预瞄点已转弯完成被删除");
			goto STATEMACHINE;
		}

		// 寻找离车辆最近的点
		nearest_waypoint_index = findNearestPoint(local_waypoints);
		if (nearest_waypoint_index == -1)
		{
			ROS_WARN("未找到离车辆最近的点");
			running_state_.store(RunStateValue::Stop);
			goto STATEMACHINE;
		}
		// 检查转弯点是否已经完成转弯，避免重复跟踪转弯点
		if (running_state_.load() == RunStateValue::Follow && local_waypoints[nearest_waypoint_index].gear == 2)
		{
			std::lock_guard<std::mutex> lock(turn_completed_mutex_);
			auto it = std::find_if(turn_completed_points_.begin(), turn_completed_points_.end(), [&](const waypoint& pt)
					{
						return fabs(local_waypoints[nearest_waypoint_index].global_x - pt.global_x) < 1e-4f &&
								fabs(local_waypoints[nearest_waypoint_index].global_y - pt.global_y) < 1e-4f;
					});

			if (it != turn_completed_points_.end())
			{
				if (nearest_waypoint_index == local_waypoints.size() - 1)
				{
					ROS_INFO("小车已到达路径终点，等待新路径......");
					running_state_.store(RunStateValue::Stop);
					goto STATEMACHINE;
				}
				else
				{
					ROS_INFO("point: (%f, %f)转弯完成, gear = %d, 跳过转弯点",
						local_waypoints[nearest_waypoint_index].global_x, local_waypoints[nearest_waypoint_index].global_y, it->gear);
					nearest_waypoint_index++;
				}
			}
		}
		if (nearest_waypoint_index >= local_waypoints.size())
		{
			nearest_waypoint_index = local_waypoints.size() - 1;
			ROS_INFO("小车已接近路径终点");
		}

		// 获取期望速度
		if (low_speed_flag_.load())
			v_expect = low_speed_;
		else
			v_expect = local_waypoints[nearest_waypoint_index].gear == 2 ? v_min_ : local_waypoints[nearest_waypoint_index].speed;
		
		// 计算预瞄点
		lookahead_waypoint_ = calculateLookaheadPoint(local_waypoints, nearest_waypoint_index, v_expect);
		ROS_INFO("预瞄点: x = %f, y = %f, yaw = %f, gear = %d, speed = %f", 
			lookahead_waypoint_.global_x, lookahead_waypoint_.global_y, lookahead_waypoint_.global_yaw, lookahead_waypoint_.gear, lookahead_waypoint_.speed);

		// 状态机
	STATEMACHINE:
		switch (running_state_.load())
		{
		case RunStateValue::Stop:
			twist_cmd.linear = 0.0;
			twist_cmd.angular = 0.0;
			publishCommand(twist_cmd);
			running_state_.store(RunStateValue::Follow);
			break;
		case RunStateValue::Follow:
			// 如果小车位置与预瞄点距离小于0.03或小车已经超过预瞄点，则小车已到达预瞄点，切换为转向状态
			if (hypot(lookahead_waypoint_.local_x, lookahead_waypoint_.local_y) < 0.03 || lookahead_waypoint_.local_x < 0.03)
			{
				twist_cmd.linear = 0.0;
				twist_cmd.angular = 0.0;
				ROS_INFO("到达预瞄点, x = %f, y = %f, yaw = %f, gear = %d", 
					lookahead_waypoint_.global_x, lookahead_waypoint_.global_y, lookahead_waypoint_.global_yaw, lookahead_waypoint_.gear);
					
				publishCommand(twist_cmd);
				running_state_.store(RunStateValue::Turn);
				break;
			}
			// pp算法计算和速度
			twist_cmd = calculate_PurePursuit(v_expect, lookahead_waypoint_);
			publishCommand(twist_cmd);
			break;
		case RunStateValue::Turn:
			// 如果小车与预瞄点角度差小于5度，则小车转弯完成，切换为跟线状态
			if (fabs(lookahead_waypoint_.local_yaw) < M_PI_4 / 9)
			{
				if (lookahead_waypoint_.gear == 2)
				{
					std::lock_guard<std::mutex> lock(turn_completed_mutex_);
					turn_completed_points_.push_back(lookahead_waypoint_);
					ROS_INFO("转弯完成, x = %f, y = %f, yaw = %f, gear = %d", 
						lookahead_waypoint_.global_x, lookahead_waypoint_.global_y, lookahead_waypoint_.global_yaw, lookahead_waypoint_.gear);
					// 满50个旧删除25个旧点
					if (turn_completed_points_.size() > 50)
						turn_completed_points_.erase(turn_completed_points_.begin(), turn_completed_points_.begin() + 25);
				}
				twist_cmd.linear = 0.0;
				twist_cmd.angular = 0.0;
				publishCommand(twist_cmd);
				running_state_.store(RunStateValue::Follow);
				break;
			}
			// 旋转
			twist_cmd.linear = 0.0;
			twist_cmd.angular = calculate_rotate_vel(lookahead_waypoint_.local_yaw);
			publishCommand(twist_cmd);
			break;
		case RunStateValue::Reverse:
			twist_cmd.linear = -0.15;
			twist_cmd.angular = 0.0;
			publishCommand(twist_cmd);
			break;
		default:
			ROS_ERROR("当前状态异常");
			twist_cmd.linear = 0.0;
			twist_cmd.angular = 0.0;
			publishCommand(twist_cmd);
			break;
		}
		loop_rate.sleep();
	}
}
