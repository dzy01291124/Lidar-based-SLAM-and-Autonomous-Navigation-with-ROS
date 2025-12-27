#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>

class SpeedController {
private:
    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::ServiceServer service;
    int speed_level;

    // 速度设置
    struct SpeedSettings {
        double linear;
        double angular;
    };

    SpeedSettings speeds[2] = {
        {0.2, 0.5},  // 低速
        {0.5, 1.0}   // 高速
    };

public:
    SpeedController() : speed_level(0) {
        // 创建Twist消息发布者
        vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

        // 创建服务
        service = nh.advertiseService("speed_toggle", &SpeedController::toggleSpeed, this);

        ROS_INFO("速度控制节点已启动，服务: /speed_toggle");
    }

    // 速度切换服务回调函数
    bool toggleSpeed(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
        // 切换速度级别
        speed_level = 1 - speed_level;
        SpeedSettings current_speed = speeds[speed_level];

        // 发布速度命令
        geometry_msgs::Twist twist;
        twist.linear.x = current_speed.linear;
        twist.angular.z = current_speed.angular;
        vel_pub.publish(twist);

        // 设置响应
        res.success = true;
        res.message = "速度已切换到级别 " + std::to_string(speed_level) + 
                      ": 线速度=" + std::to_string(current_speed.linear) + "m/s, 角速度=" + 
                      std::to_string(current_speed.angular) + "rad/s";

        ROS_INFO_STREAM(res.message);
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "speed_controller_node");
    SpeedController controller;
    ros::spin();
    return 0;
}    