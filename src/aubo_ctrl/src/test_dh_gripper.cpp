#include <iostream>
#include <ros/ros.h>
#include <dh_hand_driver/ActuateHandAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

ros::NodeHandle initRos(int argc, char** argv);

void sendGoal(string action_name, int motorID, int position, int force);

int main(int argc, char** argv) {
    ROS_INFO("----dh_hand_client----");

    // 初始化ros
    ros::NodeHandle nh = initRos(argc, argv);

    // 动作服务名称
    string action_name = "actuate_hand";

    ros::Rate rate(0.5);

    for (int i = 0; i < 3; ++i) {
        // 打开
        sendGoal(action_name, 1, 100, 100);

        rate.sleep();

        // 关闭
        sendGoal(action_name, 1, 0, 100);

        rate.sleep();
    }

    ros::spinOnce();
    ros::shutdown();
    return 0;
}

/**
 * 初始化ros
 * @param argc
 * @param argv
 * @return
 */
ros::NodeHandle initRos(int argc, char** argv) {
    ros::init(argc, argv, "dh_hand_client");
    ros::NodeHandle nh;
    return nh;
}

/**
 * 发送到指定夹爪的目的地位置
 * @param name action server name.
 * @param motorID 设备ID，默认为:1
 * @param position 夹爪位置，范围：0~100
 * @param force 力控值，范围：20~100
 */
void sendGoal(string action_name, int motorID, int position, int force) {
    actionlib::SimpleActionClient<dh_hand_driver::ActuateHandAction> client(action_name, true);

    ROS_INFO("----wait for server----");
    client.waitForServer();

    dh_hand_driver::ActuateHandGoal goal;
    goal.MotorID = motorID;
    goal.position = position;
    goal.force = force;

    ROS_INFO("----send goal----");
    client.sendGoal(goal);

    ROS_INFO("----wait for result----");
    bool result = client.waitForResult(ros::Duration(10.0));
    if(result) {
        ROS_INFO("action finished, state: %s.", client.getState().toString().c_str());
    } else {
        ROS_INFO("action don't finish before the time out.");
    }
}