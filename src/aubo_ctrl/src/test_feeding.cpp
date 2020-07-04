#include <ros/ros.h>
#include <iostream>

#include "actionlib/client/action_client.h"
#include "actionlib/client/terminal_state.h"
#include "itheima_msgs/ArmWorkAction.h"


using namespace std;


typedef actionlib::ClientGoalHandle<itheima_msgs::ArmWorkAction> ClientGoalHandle;
typedef itheima_msgs::ArmWorkFeedbackConstPtr FeedbackConstPtr;
typedef itheima_msgs::ArmWorkResultConstPtr ResultConstPtr;
typedef actionlib::ActionClient<itheima_msgs::ArmWorkAction> ActionClient;


void transition_cb(ClientGoalHandle goalHandle) {
    const actionlib::CommState &state = goalHandle.getCommState();
    if (state == actionlib::CommState::ACTIVE) {
        ROS_INFO_STREAM("ACTIVE");
    } else if (state == actionlib::CommState::DONE) {
        ROS_INFO_STREAM("DONE");
        actionlib::TerminalState ts = goalHandle.getTerminalState();
        if (ts == ts.ABORTED) {
            ROS_INFO("server working error, don't finish my job.");
        } else if (ts == ts.PREEMPTED) {
            ROS_INFO("client cancel job.");
        } else if (ts == ts.SUCCEEDED) {
            ROS_INFO("server finish job.");
        } else if (ts == ts.REJECTED) {
            ROS_INFO("server rejected job.");
        }
        ROS_INFO_STREAM(goalHandle.getResult()->result);
    } else {
        ROS_INFO_STREAM(state.toString());
    }
}

void feedback_cb(ClientGoalHandle goalHandle,
                 const FeedbackConstPtr feedback) {
    ROS_INFO("========= feedback_cb =========");
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "test_feeding");
    ros::NodeHandle node;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ActionClient client(node, "/aubo/ctrl");
    client.waitForActionServerToStart();

    itheima_msgs::ArmWorkGoal goal;
    goal.type = 0;
    auto handle = client.sendGoal(goal,
                                         boost::bind(&transition_cb, _1),
                                         boost::bind(&feedback_cb, _1, _2));


    ros::waitForShutdown();

    return 0;
}