#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <cmath>
#include <iostream>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/common/thread/thread.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

enum test_mode {
    stop_move,
    stand,
    walk,
    running,
    climb    
};

int TEST_MODE = stop_move;

class Custom {
public:
    Custom() {
        sport_client.SetTimeout(10.0f);
        sport_client.Init();

        suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
        suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);
    }

    void RobotControlCB(const std_msgs::Int32::ConstPtr &msg) {
        int new_mode = msg->data;

        // 如果状态切换，则先停止运动
        if (new_mode != TEST_MODE) {
            double time_cur = ros::Time::now().toSec();
            static double time_start = ros::Time::now().toSec();
            while(time_cur - time_start <= 2){
                time_cur = ros::Time::now().toSec();
                sport_client.Move(0.0, 0.0, 0.0);
            }
            StopMovement();  // 停止机器人
            sleep(3);
            TEST_MODE = new_mode;
            SwitchGait(TEST_MODE);
        }
    }

    void CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
        // 只有在行走或跑步模式下使用cmd_vel控制
        double time_cur = ros::Time::now().toSec();
        static double time_cmd = ros::Time::now().toSec();
        if (TEST_MODE == walk || TEST_MODE == running || TEST_MODE == climb) {
            double linear_x = msg->linear.x;
            double linear_y = msg->linear.y;
            double angular_z = msg->angular.z;
            if(sqrt(pow(linear_x, 2.0) + pow(linear_y, 2.0) + pow(angular_z, 2.0)) > 0.01){
                time_cmd = ros::Time::now().toSec();
            }

            if(time_cur - time_cmd <= 0.1){
                sport_client.Move(linear_x, linear_y, angular_z);
                std::cout<<linear_x<<linear_y<<angular_z<<std::endl;
            }
        }
    }

    void SwitchGait(int mode) {

        if (mode < stop_move || mode > climb) {
        std::cout << "Received invalid mode: " << mode << ". Stopping robot." << std::endl;
        StopMovement();  // 停止机器人
        return; // 退出方法，不处理状态切换
        }


        switch (mode) {
            case stand:
                std::cout << "Switching to stand" << std::endl;
                sport_client.RecoveryStand();
                break;
            case walk:
                std::cout << "Switching to walk" << std::endl;
                sport_client.SwitchGait(1);
                break;
            case running:
                std::cout << "Switching to running" << std::endl;
                sport_client.SwitchGait(2);
                break;
            case climb:
                std::cout << "Switching to climb" << std::endl;
                sport_client.SwitchGait(3);
                break;
            case stop_move:
            default:
                std::cout << "Stop moving" << std::endl;
                StopMovement();
                break;
        }
    }

    void GetInitState()
    {
        px0 = state.position()[0];
        py0 = state.position()[1];
        yaw0 = state.imu_state().rpy()[2];
        std::cout << "initial position: x0: " << px0 << ", y0: " << py0 << ", yaw0: " << yaw0 << std::endl;
    };

    void StopMovement() {
        std::cout << "Stopping robot movement" << std::endl;
        sport_client.StopMove();
    }

    void HighStateHandler(const void *message) {
        state = *(unitree_go::msg::dds_::SportModeState_ *)message;
    }

    unitree_go::msg::dds_::SportModeState_ state;
    unitree::robot::go2::SportClient sport_client;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

    double px0, py0, yaw0; // 初始状态的位置和偏航
};

int main(int argc, char **argv) {
    // if (argc < 2) {
    //     unitree::robot::ChannelFactory::Instance()->Init(1, "lo");
    // } else {
    //     unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    // }
    unitree::robot::ChannelFactory::Instance()->Init(0, "enp114s0");
    ros::init(argc, argv, "motion_controller");

    Custom custom;

    sleep(1); // 等待 1 秒以便获取稳定状态

    custom.GetInitState(); // 获取初始位置

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::Subscriber state_sub = nh.subscribe("/sport_mode", 10, &Custom::RobotControlCB, &custom);
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &Custom::CmdVelCallback, &custom); // 订阅cmd_vel

    ros::spin();
    return 0;
}
