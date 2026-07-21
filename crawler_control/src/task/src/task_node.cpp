#include <ros/ros.h>
#include <vector>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string.h>
#include <algorithm>
// android msg
#include "mower_msgs/TaskStatus.h"
#include "mower_msgs/CheckResult.h"
#include "mower_msgs/Manual_Set.h"
#include "mower_msgs/Manual_Driving_Cmd.h"
#include "mower_msgs/Fault_Code.h"
#include "mower_msgs/Direct_Control.h"
#include "mower_msgs/VehicleCmd.h"

// check msg
#include "mower_msgs/LidarSelfDtect.h"
#include "mower_msgs/PerceptionSelfDetect.h"
#include "mower_msgs/MultiMapSelfDetect.h"
#include "mower_msgs/PlaningOK.h"
#include "mower_msgs/PlanType.h"
#include "mower_msgs/CamerargbState.h"
#include "mower_msgs/SegState.h"
#include "mower_msgs/ControlOk.h"
#include "mower_msgs/ControlState.h"
#include "mower_msgs/GnssOK.h"
#include "mower_msgs/VslamState.h"
#include "mower_msgs/CameraState.h"
//@ xhj： add
#include "mower_msgs/Position.h"
#include "mower_msgs/Monitor.h"
#include "util/LocalPose.h"
#include "mower_msgs/LocalPath.h"
#include "nav_msgs/Path.h"
#include <tf/transform_datatypes.h>
// #include "util/VehicleControl.h"  // 添加VehicleControl消息头文件

#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <cmath> // 添加数学函数库

mower_msgs::TaskStatus status;
mower_msgs::Fault_Code fault;
mower_msgs::Monitor monitor;

using namespace std;
// check sub
ros::Subscriber sub_camera; //
ros::Subscriber sub_vslam;  //
ros::Subscriber sub_camerargb;
ros::Subscriber sub_seg;
ros::Subscriber sub_lidar;      //
ros::Subscriber sub_perception; //
ros::Subscriber sub_gnss;
ros::Subscriber sub_planche;
ros::Subscriber sub_ctrlche;
ros::Subscriber sub_multimap; //
//ros::Subscriber sub_vehistatus;

// pub android
ros::Publisher pub_status;
ros::Publisher pub_result;
ros::Publisher pub_fault;
ros::Publisher pub_monitor;
ros::Publisher pub_direct_control;
ros::Publisher pub_vehicle_cmd;

// sub android
//ros::Subscriber sub_manual;
ros::Subscriber sub_manual_driving;
ros::Subscriber sub_singal;

ros::Subscriber sub_warn;
ros::Publisher pub_stopflag;

bool check_flag = false; // 自检标志
bool manual_better = false;
std_msgs::Bool stop_car;

// 初始化程序相关变量
bool init_mode = false;       // 初始化模式标志
bool has_position = false;    // 是否有定位数据
ros::Time init_start_time;    // 初始化开始时间
//double figure8_radius = 1.5;  // "8"字形半径（米）
//double figure8_period = 20.0; // "8"字形周期（秒）

// 初始化确认相关变量
bool init_requested = false;     // 是否请求初始化
bool init_confirmed = false;     // 是否确认初始化
ros::Publisher pub_init_request; // 发布初始化请求

// init check status
bool check_gnss = false;
bool check_fusion = false;
bool check_cam = false;
bool check_vslam = false;
bool check_lidar = false;
bool check_perception = false;
bool check_multimap = false;
//bool check_battery = false;
//bool check_warn = false;
bool check_camerargb = true;
bool check_planok = false;
bool check_controlok = false;
bool check_seg = false;

// safe warning  *stop car
int rangewarn_flag = 0; // 范围停车预警
bool front_flag = false;
bool rear_flag = false;
bool yolofront_flag = false;
bool rollover_flag = false; // 侧翻状态
bool outboundary_flag = false;
bool lowpower_flag = false;  // 低电量 现定义-30%
bool appsignal_flag = false; // 默认  /signal == pause | stop  1; continue  0;
bool init_finish = false;

// bool current_has_position = false; // 当前是否有定位数据
bool turn_left_done = false;
ros::Time turn_time;
bool turn_active = false;

// 定义标志位对应的字符串表
const std::string node_names[13] = {
    "Battery_soc",     // 无，去除
    "Chassis_warning", // 无
    "FusionMap",
    "Vslam",
    "Camerargb",
    "Seg",
    "Lidar",
    "Perception",
    "Gnss",
    "Planning",
    "Control",
    "Multimap",
    "Camera"};

enum tasks
{
    Holding,
    Working,                 // 工作状态
    Local_path_error,        // 绕障路径异常
    Vehicle_rollover,        // 侧翻状态
    Returning,               // 返回
    Pausing,                 // 暂停工作
    Self_checking,           // 自检状态
    Self_check_fault,        // 自检失败
    Obstacle_parking,        // 遇障停车
    Passing_connecting_space // 通过连通区
} taskstatus;

namespace {
    enum class SignalCmd {
        InitLocation,
        InitConfirm,
        InitCancel,
        Reset,
        StartExecution,
        StopExecution,
        StartWork,
        Pause,
        Continue,
        Return,
        Stop,
        Unknown
    };

    SignalCmd ParseSignal(const std::string& signal)
    {
        if (signal == "init_location")   return SignalCmd::InitLocation;
        if (signal == "true")            return SignalCmd::InitConfirm;
        if (signal == "false")           return SignalCmd::InitCancel;
        if (signal == "reset")           return SignalCmd::Reset;
        if (signal == "start_execution") return SignalCmd::StartExecution;
        if (signal == "stop_execution")  return SignalCmd::StopExecution;
        if (signal == "start_work")      return SignalCmd::StartWork;
        if (signal == "pause")           return SignalCmd::Pause;
        if (signal == "continue")        return SignalCmd::Continue;
        if (signal == "return")          return SignalCmd::Return;
        if (signal == "stop")            return SignalCmd::Stop;
        return SignalCmd::Unknown;
    }
}

void goStraight()
{
    mower_msgs::VehicleCmd cmd_msg;
    cmd_msg.turn_value = 0;
    cmd_msg.drive_value = 100;
    cmd_msg.ad_control_enable = 1;
    cmd_msg.gear_model = 3;
    cmd_msg.mover_bool = 0;
    cmd_msg.mower_height = 1;
    cmd_msg.header.stamp = ros::Time::now(); // 设置时间戳
    pub_vehicle_cmd.publish(cmd_msg);
}
void controlFigure8_turnleft()
{
    mower_msgs::VehicleCmd cmd_msg;
    cmd_msg.drive_value = 100;
    cmd_msg.turn_value = -20;                // 向左转，持续走圆
    cmd_msg.ad_control_enable = 1;           // 自动驾驶控制开启
    cmd_msg.gear_model = 3;                  // 前进档
    cmd_msg.mover_bool = 0;                  // 不割草
    cmd_msg.mower_height = 1;                // 割草高度（随便设一个）
    cmd_msg.header.stamp = ros::Time::now(); // 时间戳
    pub_vehicle_cmd.publish(cmd_msg);
    ROS_INFO("Driving in circle: drive_value=%d, turn_value=%d",
             cmd_msg.drive_value, cmd_msg.turn_value);
}
void controlFigure8_turnright()
{
    mower_msgs::VehicleCmd cmd_msg;
    cmd_msg.drive_value = 100;
    cmd_msg.turn_value = 20;                 // 向右转，持续走圆
    cmd_msg.ad_control_enable = 1;           // 自动驾驶控制开启
    cmd_msg.gear_model = 3;                  // 前进档
    cmd_msg.mover_bool = 0;                  // 不割草
    cmd_msg.mower_height = 1;                // 割草高度（随便设一个）
    cmd_msg.header.stamp = ros::Time::now(); // 时间戳
    pub_vehicle_cmd.publish(cmd_msg);
    ROS_INFO("Driving in circle: drive_value=%d, turn_value=%d",
             cmd_msg.drive_value, cmd_msg.turn_value);
}

void controlFigure8_turn()
{
    if(!turn_left_done)
    {
        if ((ros::Time::now() - turn_time).toSec() < 5.0) // 左转5s
        {
            controlFigure8_turnleft();
            ROS_INFO("Turn left for %.2f seconds", (ros::Time::now() - turn_time).toSec());
        }
    }
    else
    {
        if ((ros::Time::now() - turn_time).toSec() < 5.0) // 左转5s
        {
            controlFigure8_turnright();
            ROS_INFO("Turn right for %.2f seconds", (ros::Time::now() - turn_time).toSec());
        }
    }
}

void parking()
{
    cout << "~~~stop~~~ flag front " << yolofront_flag << " ,rear " << rear_flag << " ,appsignal_flag " << appsignal_flag << " ,outboundary_flag " << outboundary_flag << endl;
    if (!rollover_flag && !yolofront_flag && !appsignal_flag && !outboundary_flag)
    {
        stop_car.data = false; // all false
    }
    else
    {
        stop_car.data = true;
    }
    pub_stopflag.publish(stop_car);
}

// 检查所有标志位是否为true，并处理不符合的标志位
bool checkAndResetNodes(const mower_msgs::Monitor &msgs)
{
    //bool all_true = true;
    monitor = msgs;
    pub_monitor.publish(monitor);

    bool all_true = std::find(monitor.node_normal.begin(),
                              monitor.node_normal.end(), false) == monitor.node_normal.end();
    // 将所有标志位 置为false
    monitor.node_normal.fill(false);

    // 如果所有节点都为true，执行相应操作
    return true;
    // if (1)
    // { //  all_true
    //     return true;
    // }
    // else
    // {
    //     return false;
    // }
}

void task_run()
{
    switch (taskstatus)
    {
    case Holding:
        status.task_status = "Holding";
        break;
    case Working:
        status.task_status = "Working";
        break;
    case Local_path_error:
        break;
    case Vehicle_rollover:
        break;
    case Returning:
        status.task_status = "Returning";
        break;
    case Pausing:
        status.task_status = "Pausing";
        break;
    case Self_checking:
        status.task_status = "Self_checking";
        break;
    case Self_check_fault:
        status.task_status = "Self_check_fault";
        break;
    case Obstacle_parking:
        status.task_status = "Obstacle_parking";
        break;
    case Passing_connecting_space:
        status.task_status = "Passing_connecting_space";
        break;
    default:
        break;
    }
    pub_status.publish(status);
}

pid_t pid_controller_pid = -1; // 全局变量，记录进程 PID

void ManualDriveCallBack(const mower_msgs::Manual_Driving_Cmd &manual_drive_msgs)
{
    mower_msgs::VehicleCmd vehicle_cmd;
    vehicle_cmd.ad_control_enable = manual_drive_msgs.ad_control_enable;
    vehicle_cmd.drive_value = manual_drive_msgs.drive_value;
    vehicle_cmd.turn_value = manual_drive_msgs.turn_value;
    vehicle_cmd.mover_bool = manual_drive_msgs.mover_bool;
    vehicle_cmd.gear_model = manual_drive_msgs.gear_model;
    vehicle_cmd.mower_height = manual_drive_msgs.mow_height;
    pub_vehicle_cmd.publish(vehicle_cmd);
}

void SingalCallBack(const std_msgs::String &singal_msgs)
{
    switch (ParseSignal(singal_msgs.data))
    {
        case SignalCmd::InitLocation:
        {
            init_requested = true;
            init_confirmed = false;
            init_mode = false;

            std_msgs::Bool init_request_msg;
            init_request_msg.data = true;
            pub_init_request.publish(init_request_msg);

            ROS_INFO("Initialization requested - waiting for confirmation");
            break;
        }
        case SignalCmd::InitConfirm:
        {
            turn_left_done = false;
            turn_active = false;
            if (init_requested)
            {
                init_confirmed = true;
                init_mode = true;
                init_start_time = ros::Time::now();
                init_finish = false;
                ROS_INFO("Initialization confirmed - starting Figure8 pattern");
            }
            else
            {
                ROS_WARN("No initialization request or position already available");
            }
            break;
        }
        case SignalCmd::InitCancel:
        {
            if (init_mode)
            {
                mower_msgs::VehicleCmd cmd_msg;
                cmd_msg.turn_value = 0;
                cmd_msg.drive_value = 0;
                cmd_msg.ad_control_enable = 1;
                cmd_msg.gear_model = 0;
                cmd_msg.mover_bool = 0;
                cmd_msg.mower_height = 0;
                cmd_msg.header.stamp = ros::Time::now();
                pub_vehicle_cmd.publish(cmd_msg);

                init_mode = false;
                init_requested = false;
                init_confirmed = false;
                init_finish = false;

                turn_left_done = false;
                turn_active = false;

                ROS_INFO("Initialization cancelled - stopping vehicle");
            }
            break;
        }
        case SignalCmd::Reset:
        {
            turn_left_done = false;
            turn_active = false;
            init_finish = false;

            init_mode = false;
            init_requested = false;
            init_confirmed = false;
            break;
        }
        case SignalCmd::StartExecution:
        {
            if (pid_controller_pid == -1)
            {
                pid_controller_pid = fork();
                if (pid_controller_pid == 0)
                {
                    setsid();
                    execl("/bin/bash", "bash", "-c",
                          "source /home/nvidia/crawler_control/devel/setup.bash && "
                          "roslaunch pure_pursuit pure_pursuit.launch > /home/nvidia/crawler_control/logs/pure_pursuit.logs 2>&1 &",
                          (char *)0);
                    perror("execl failed");
                    exit(1);
                }
                else if (pid_controller_pid > 0)
                {
                    ROS_INFO("Started pid_controller.launch with PID: %d", pid_controller_pid);
                }
                else
                {
                    ROS_ERROR("Failed to fork process for roslaunch");
                    pid_controller_pid = -1;
                }
            }
            else
            {
                ROS_WARN("pp_controller already running with PID: %d", pid_controller_pid);
            }
            break;
        }
        case SignalCmd::StopExecution:
        {
            if (pid_controller_pid > 0)
            {
                int ret = system("rosnode kill /pure_pursuit");
                if (ret == 0)
                {
                    ROS_INFO("Sent rosnode kill /pure_pursuit");
                }
                kill(-pid_controller_pid, SIGKILL);

                int status;
                for (int i = 0; i < 50; ++i)
                {
                    if (waitpid(pid_controller_pid, &status, WNOHANG) == pid_controller_pid)
                        break;
                    ros::Duration(0.1).sleep();
                }
                pid_controller_pid = -1;
            }
            else
            {
                ROS_WARN("No running pid_controller to stop");
            }
            break;
        }
        case SignalCmd::StartWork:
        {
            taskstatus = Working;
            manual_better = false;
            appsignal_flag = false;
            break;
        }
        case SignalCmd::Pause:
        {
            taskstatus = Pausing;
            manual_better = true;
            appsignal_flag = true;
            break;
        }
        case SignalCmd::Continue:
        {
            taskstatus = Working;
            manual_better = false;
            appsignal_flag = false;
            break;
        }
        case SignalCmd::Return:
        {
            taskstatus = Returning;
            manual_better = false;
            break;
        }
        case SignalCmd::Stop:
        {
            appsignal_flag = true;
            break; 
        }    
        case SignalCmd::Unknown:
        {
             ROS_WARN("Unknown signal appears");
            break; 
        }    
    }
}

//@xhj add FusionMAP state to fault_code[2]
void FusionMapCallBack(const mower_msgs::Position &msgs)
{
    check_fusion = (msgs.position_state == 1 || msgs.position_state == 2 || msgs.position_state == 5) ? 1 : 0;
    fault.fault_code[2] = check_fusion ? "0" : "1";
    monitor.node_normal[2] = check_fusion ? 1 : 0;

    // 检测定位状态
    has_position = check_fusion; //(msgs.position_state == -1);//??????

    // 如果从无定位变为有定位，且正在初始化模式，则退出初始化
    if (has_position && init_mode)
    {
        init_mode = false;
        init_requested = false;
        init_confirmed = false;
        turn_active = false;
        ROS_INFO("Position acquired! Exiting initialization mode.");
        ROS_INFO("Stopping vehicle after acquiring position");
    }

    if (has_position && !init_finish)
    {
        init_finish = true;
    }
}
void VslamCallBack(const mower_msgs::VslamState &vslam_msgs)
{
    check_vslam = vslam_msgs.is_vslam_ok;
    fault.fault_code[3] = check_vslam ? "0" : "1";
}

void CameraRGBCallBack(const mower_msgs::CamerargbState &camerargb_msgs)
{
    check_camerargb = camerargb_msgs.is_camerargb_ok;
    fault.fault_code[4] = check_camerargb ? "0" : "1";
}

void SegCallBack(const mower_msgs::SegState &seglok_msgs)
{
    check_seg = seglok_msgs.is_seg_ok;
    fault.fault_code[5] = check_seg ? "0" : "1";
}

void LidarCallBack(const mower_msgs::LidarSelfDtect &lidar_msgs)
{
    check_lidar = lidar_msgs.is_lidar_ok;
    fault.fault_code[6] = check_lidar ? "0" : "1";
}

void PerceptionCallBack(const mower_msgs::PerceptionSelfDetect &perception_msgs)
{
    check_perception = perception_msgs.is_perception_ok;
    fault.fault_code[7] = check_perception ? "0" : "1";
}

void GnssCallBack(const mower_msgs::GnssOK &gnssok_msgs)
{
    check_gnss = gnssok_msgs.is_gnss_ok;
    fault.fault_code[8] = check_gnss ? "0" : "1";
}

void PlanCheckCallBack(const mower_msgs::PlaningOK &planok_msgs)
{
    check_planok = planok_msgs.is_planing_ok;
    fault.fault_code[9] = check_planok ? "0" : "1";
}

void CtrlCheckCallBack(const mower_msgs::ControlOk &controlok_msgs)
{
    check_controlok = controlok_msgs.is_control_ok;
    fault.fault_code[10] = check_controlok ? "0" : "1";
    monitor.node_normal[10] = check_controlok ? 1 : 0;
}

void MultiMapCallBack(const mower_msgs::MultiMapSelfDetect &multimap_msgs)
{
    check_multimap = multimap_msgs.is_multi_map_ok;
    fault.fault_code[11] = check_multimap ? "0" : "1";
}

void CamCallBack(const mower_msgs::CameraState &cam_msgs)
{
    check_cam = cam_msgs.is_camera_ok;
    fault.fault_code[12] = check_cam ? "0" : "1";
}

void ImuCallBack(const std_msgs::Bool &imu_msgs)
{
    if (!imu_msgs.data)
    {
        rollover_flag = true;
    }
    else
    {
        rollover_flag = false;
    }
}

void OutBoundaryCallBack(const std_msgs::Bool &msgs)
{
    if (msgs.data)
    {
        outboundary_flag = true;
    }
    else
    {
        outboundary_flag = false;
    }
}

//** @xhj: add range warning **//
void SpeedInfoCallBack(const util::LocalPose &v_pose)
{
    if (v_pose.vehicle_speed > 0)
    {
        rangewarn_flag = 3;
    }
    else if (v_pose.vehicle_speed < 0)
    {
        rangewarn_flag = 1;
    }
    else
    {
        rangewarn_flag = 0;
    }
}
//@xhj 2503  yolov8
void YoloflagfrontCallBack(const std_msgs::Bool &range_msgs)
{
    if (range_msgs.data && rangewarn_flag != 1)
    {
        yolofront_flag = true;
        ROS_WARN("!!! YOLO found dynamic obstalces !!!");(!imu_msgs.data)
    }
    else
    {
        yolofront_flag = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_node");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("task start");

    sub_manual_driving = nh.subscribe("/mower/manual_driving_cmd", 1, ManualDriveCallBack);
    sub_singal = nh.subscribe("/signal", 1, SingalCallBack); //@app 信号 单次下发
    // check sub
    ros::Subscriber sub_fusionmap = nh.subscribe("/Mower/position", 1, FusionMapCallBack);
    sub_camera = nh.subscribe("/mower/camera/state", 1, CamCallBack);
    sub_vslam = nh.subscribe("/mower/vslam_state", 1, VslamCallBack);
    sub_lidar = nh.subscribe("/mower/lidar_ok", 1, LidarCallBack);
    sub_perception = nh.subscribe("/mower/perception_ok", 1, PerceptionCallBack);
    sub_multimap = nh.subscribe("/mower/multimap_ok", 1, MultiMapCallBack);
    sub_camerargb = nh.subscribe("/mower/camerargb_ok", 1, CameraRGBCallBack);
    sub_seg = nh.subscribe("/mower/seg_ok", 1, SegCallBack);
    sub_gnss = nh.subscribe("/mower/gnss_ok", 1, GnssCallBack);
    sub_planche = nh.subscribe("/mower/planing_ok", 1, PlanCheckCallBack);
    sub_ctrlche = nh.subscribe("/mower/control_ok", 1, CtrlCheckCallBack);

    // sub safe
    ros::Subscriber sub_speedinfo = nh.subscribe("/nanobot/localpose", 1, SpeedInfoCallBack);                 //@ 履带车接 定位速度反馈
    ros::Subscriber sub_yolofront = nh.subscribe("/YoloSeg/yolocontrol_publisher", 1, YoloflagfrontCallBack); //@xhj 2503
    ros::Subscriber sub_imu = nh.subscribe("/Mower/car_state", 1, ImuCallBack);
    ros::Subscriber sub_outboundary = nh.subscribe("/mower/stop_car1", 1, OutBoundaryCallBack);

    // android->control
    pub_direct_control = nh.advertise<mower_msgs::Direct_Control>("/mower/direct_control", 1);
    pub_vehicle_cmd = nh.advertise<mower_msgs::VehicleCmd>("/vehicle/cmd", 1); //@xhj 2406
    // pub_vehicle_control = nh.advertise<util::VehicleControl>("/vehicle/cmd", 1);  // 修改为正确的話題
    pub_init_request = nh.advertise<std_msgs::Bool>("/init_request", 1); // 添加初始化请求发布器
    // task <-> Android
    pub_status = nh.advertise<mower_msgs::TaskStatus>("/mower/task_status", 1);
    pub_result = nh.advertise<mower_msgs::CheckResult>("/mower/check_result", 1);
    pub_monitor = nh.advertise<mower_msgs::Monitor>("/mower/monitor", 1); //@xhj
    pub_stopflag = nh.advertise<std_msgs::Bool>("/mower/stop_car", 1);

    // 初始化所有监控节点为false(异常状态)，节点正常为true
    for (int i = 0; i < 13; ++i)
    {
        monitor.node_normal[i] = false;
    }

    stop_car.data = false;
    double time = ros::Time::now().toSec();
    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        ros::spinOnce();

        ROS_INFO("init_mode=%d, has_position=%d, init_finish=%d", init_mode, has_position, init_finish);

        if (init_finish)
        {
            if (ros::Time::now().toSec() - time > 5.0)
            {
                std_msgs::Bool init_request_msg;
                init_request_msg.data = false;
                pub_init_request.publish(init_request_msg);
                time = ros::Time::now().toSec();
            }
        }

        // 如果在初始化模式，执行"8"字形控制
        if (init_mode && !has_position)
        {
            // 先sleep 3秒
            if (!turn_active)
            {
                ROS_INFO("Waiting 3 seconds before starting straight driving...");
                ros::Duration(3.0).sleep(); // 
                turn_active = true;
                turn_time = ros::Time::now();
            }

            if ((ros::Time::now() - turn_time).toSec() < 7.0 && (ros::Time::now() - turn_time).toSec() >= 5.0) // 判断是否直行
            {
                goStraight();
                ROS_INFO("Go Straight for %.2f seconds", (ros::Time::now() - turn_time).toSec());
            }
            else if((ros::Time::now() - turn_time).toSec() >= 7.0)// 判断是否完成直行
            {
                turn_time = ros::Time::now();//重新计时
                turn_left_done = !turn_left_done;//交换转弯方向
                controlFigure8_turn()
            } 
            else 
            {
                controlFigure8_turn();
            }
        }

        mower_msgs::CheckResult result;
        static auto t0 = ros::Time::now();
        ros::Time t1 = ros::Time::now();
        double check_time = (t1 - t0).toSec();
        static bool check_once = 0;
        if (check_time < 5.0)
        {
            if (std::fmod(check_time, 2.0) < 0.1)
            {
                ROS_INFO("~check~ time : %.2f ===> %d ", check_time, check_flag);
            }
            taskstatus = Self_checking;
        }
        else if (!check_once)
        {
            if (!check_flag)
            {
                ROS_INFO("~mointor~ check ==>Fail");
                taskstatus = Self_check_fault;
                result.is_checkresult_ok = false;
            }
            else
            {
                ROS_INFO("~mointor~ check ok");
                taskstatus = Holding;
                result.is_checkresult_ok = true;
            }
            check_once = 1;
            task_run();
            pub_status.publish(status);
            pub_result.publish(result);
        }

        task_run();
        parking(); //@xhj：2406
        check_flag = checkAndResetNodes(monitor);
        loop_rate.sleep();
    }

    return 0;
}
