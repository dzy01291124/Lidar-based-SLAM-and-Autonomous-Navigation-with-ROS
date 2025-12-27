#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <termios.h>
#include <unistd.h>

// 获取键盘输入的函数
char getch() {
    char ch;
    struct termios oldt, newt;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_input_node");
    ros::NodeHandle nh;

    // 创建服务客户端
    ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("speed_toggle");

    // 等待服务可用
    ROS_INFO("等待speed_toggle服务...");
    ros::service::waitForService("speed_toggle");
    ROS_INFO("服务已连接");

    // 显示控制说明
    std::cout << "\n===== 机器人速度控制 =====" << std::endl;
    std::cout << "按 [t] 切换速度级别" << std::endl;
    std::cout << "按 [q] 退出\n" << std::endl;

    std_srvs::Trigger srv;
    char key;

    while (ros::ok()) {
        key = getch();

        if (key == 't') {
            if (client.call(srv)) {
                ROS_INFO("速度切换结果: %s", srv.response.message.c_str());
            } else {
                ROS_ERROR("服务调用失败");
            }
        } else if (key == 'q') {
            ROS_INFO("退出程序");
            break;
        }

        ros::spinOnce();
    }

    return 0;
}    