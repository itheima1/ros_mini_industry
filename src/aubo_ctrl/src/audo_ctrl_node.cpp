// aubo 下料操作节点

#include <ros/ros.h>
#include <iostream>

#include <Robot.h>
#include <BlockingQueue.h>
#include <actionlib/server/action_server.h>
#include <itheima_msgs/ArmWorkAction.h>
#include <itheima_msgs/GetBoxPoses.h>

#include <thread>
#include <actionlib/server/server_goal_handle.h>

// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
//Eigen 几何模块
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv/cxeigen.hpp>

#include <Rotation3DUtils.h>

#include <dh_hand_driver/ActuateHandAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;
using namespace cv;
using namespace Eigen;

typedef actionlib::ServerGoalHandle<itheima_msgs::ArmWorkAction> ServerGoalHandle;
typedef actionlib::ActionServer<itheima_msgs::ArmWorkAction> ActionServer;


//消息队列
BlockingQueue<ServerGoalHandle> handleQueue;
map<string, bool> canceledIds;
Mat exMat;
Mat cameraMatrix, distCoeffs;


void goal_cb(ServerGoalHandle handle) {
    handleQueue.put(handle);
}

void cancel_cb(ServerGoalHandle handle) {

}

bool ctrl_gripper(int position, int force){
    actionlib::SimpleActionClient<dh_hand_driver::ActuateHandAction> client("actuate_hand", true);

    ROS_INFO("----wait for server----");
    client.waitForServer();

    dh_hand_driver::ActuateHandGoal goal;
    goal.MotorID = 1;
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
    return result;
}

bool open_gripper() {
    return ctrl_gripper(100, 100);
}

bool close_gripper() {
    return ctrl_gripper(30, 60);
}

Point3d getCameraXYZ(Point_<int> point, Mat cameraMatrix, double z_distance) {
    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);

    int u = point.x; // 列
    int v = point.y; // 行

    double z = z_distance / 1000.0;
    double x = (u - cx) * z / fx;
    double y = (v - cy) * z / fy;
    return cv::Point3d(x, y, z);
}


Mat getExMat(cv::String exCailFilePath) {
    double angle = 0;
    double axisX = 0;
    double axisY = 0;
    double axisZ = 0;
    double translationX = 0;
    double translationY = 0;
    double translationZ = 0;

    // 使用opencv读取文件
    FileStorage fs(exCailFilePath, FileStorage::READ);
    fs["Angle"] >> angle;
    fs["AxisX"] >> axisX;
    fs["AxisY"] >> axisY;
    fs["AxisZ"] >> axisZ;
    fs["TranslationX"] >> translationX;
    fs["TranslationY"] >> translationY;
    fs["TranslationZ"] >> translationZ;
    // 轴角对
    Vector3d axisMatrix(axisX, axisY, axisZ);
    AngleAxisd angleAxisd(angle, axisMatrix);
    // 获取旋转矩阵
    Matrix3d rotationMatrix = angleAxisd.toRotationMatrix();
    // 获取平移矩阵
    Vector3d translationMatrix(translationX, translationY, translationZ);
    translationMatrix = translationMatrix / 1000;

    cout << "旋转矩阵：\n " << angleAxisd.toRotationMatrix() << endl;
    cout << "平移向量：\n" << translationMatrix << endl;

    Mat_<double> ex_rotationMat(3, 3);
    eigen2cv(rotationMatrix, ex_rotationMat);
    Mat_<double> ex_translationMat(3, 1);
    eigen2cv(translationMatrix, ex_translationMat);

    Mat exMat = toHomogeneousMat(ex_rotationMat, ex_translationMat);
    return exMat;
}

Mat getBoxMat(const vector<int> &center, const vector<int> &vect, double z_distance, bool isFromVectorY = false) {// 将二维像素坐标转成三维坐标
    Point3d point_center = getCameraXYZ(Point2i(center[0], center[1]), cameraMatrix, z_distance);

    Point3d point_refer = getCameraXYZ(Point2i(center[0] + vect[0], center[1] + vect[1]), cameraMatrix, z_distance);
    Point3d vector_refer = point_refer - point_center;
    Vector3d xVec, yVec, zVec = Vector3d(0, 0, 1);

    // Z方向
    zVec = zVec.normalized();

    if (isFromVectorY) {
        // 1.Y方向， 并归一化
        yVec = Vector3d(vector_refer.x, vector_refer.y, vector_refer.z);
        yVec = yVec.normalized();
        // 2.X方向，通过叉乘Y x Z
        xVec = yVec.cross(zVec);
        xVec = xVec.normalized();

    }else {
        // 1.X方向, 并归一化
        xVec = Vector3d(vector_refer.x, vector_refer.y, vector_refer.z);
        xVec = xVec.normalized();
        // 2.Y方向，通过叉乘Z x X
        yVec = zVec.cross(xVec);
        yVec = yVec.normalized();
    }

    // 5.根据三个坐标轴，可以构建一个旋转矩阵，

    Mat_<double> box_R_Mat = (Mat_<double>(3, 3)
            << xVec[0], yVec[0], zVec[0],
            xVec[1], yVec[1], zVec[1],
            xVec[2], yVec[2], zVec[2]);

    Mat_<double> box_t_Mat = (Mat_<double>(3, 1)
            << point_center.x, point_center.y, point_center.z);
    Mat box2CameraMat = toHomogeneousMat(box_R_Mat, box_t_Mat);
    return box2CameraMat;
}

// 定义工具位姿 ----------------------------------------------------- ④
//    double tool_x = 0, tool_y = 0, tool_z = 80;
double tool_x = 0, tool_y = 0, tool_z = 170;//大环夹爪
Mat_<double> toolMat = (Mat_<double>(4, 4) <<
        1, 0, 0, tool_x / 1000,
        0, 1, 0, tool_y / 1000,
        0, 0, 1, tool_z / 1000,
        0, 0, 0, 1
);

void do_feeding(ServerGoalHandle &handle) {
    // 上料
    //1. 获取图片，并解析图片，获得位置和姿态信息
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<itheima_msgs::GetBoxPoses>("/box/poses");
    client.waitForExistence();
    itheima_msgs::GetBoxPoses service;
    client.call(service);
    auto agv_poses = service.response.agv_poses;
    client.shutdown();

    std::cout << "--------------------------------------1" << std::endl;
    if (agv_poses.empty()) {
        // 没有盒子
        itheima_msgs::ArmWorkResult result;
        result.result = "No box checked";
        handle.setRejected(result);
        return;
    }

    std::cout << "--------------------------------------2" << std::endl;
    auto first_pose = agv_poses[agv_poses.size() - 1];
    if (first_pose.type != 0) {
        // 有盒子但是最右边的不是原料区的盒子
        itheima_msgs::ArmWorkResult result;
        result.result = "No source box checked";
        handle.setRejected(result);
        return;
    }
    std::cout << "--------------------------------------3" << std::endl;

    // 激活
    handle.setAccepted();
    itheima_msgs::ArmWorkResult result;

    std::vector<int> center = first_pose.center;
    std::vector<int> vect = first_pose.vect;

    // 取出盒子位姿T1，转成4x4齐次矩阵 ------------------------------------- ②
    // 用二维坐标构建三维位姿
    Mat box2CameraMat = getBoxMat(center, vect, 1100, true);

    const Mat &toolMatInv = homogeneousInverse(toolMat);
    cout << "exMat:\n" << exMat << endl;
    cout << "box2CameraMat:\n" << box2CameraMat << endl;
    // 测试模板抓取  `⑤ = ③ · ② · inv(④)`
    Mat_<double> finalMat = exMat * box2CameraMat * toolMatInv;

    double *pos = convert2pose(finalMat);

    // ----------------------------------定义z后退12cm的位置
    Mat_<double> toolMatUp;
    toolMat.copyTo(toolMatUp);
    toolMatUp.at<double>(2, 3) += 0.12f; // z增加12cm
    const Mat &toolMatUpInv = homogeneousInverse(toolMatUp);
    Mat_<double> finalMatUp = exMat * box2CameraMat * toolMatUpInv;
    double *posUp = convert2pose(finalMatUp);

    cout << "x: " << pos[0] << " y: " << pos[1] << " z: " << pos[2];
    cout << " r: " << pos[3] * RA2DE << " p: " << pos[4] * RA2DE << " y: " << pos[5] * RA2DE << endl;

//    if (true) { return; }

    aubo_robot_namespace::JointParam jointParam;
    Robot::getInstance()->robotServiceGetJointAngleInfo(jointParam);

    // 先MoveJ移动到上方12cm位置
    int rst = Robot::getInstance()->moveJwithPose(posUp, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "移动到上方失败" << endl;

        result.result = "移动到上方失败";
        handle.setAborted(result);

        return;
    }
    // 打开夹爪
    open_gripper();

    // 再MoveL下降放到正确位置（抓取）
    rst = Robot::getInstance()->moveL(pos, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "moveL下降到位失败" << endl;

        result.result = "moveL下降到位失败";
        handle.setAborted(result);

        return;
    }

    // 闭合夹爪，夹取盒子
    close_gripper();

    // 后MoveL上升到12cm位置
    rst = Robot::getInstance()->moveL(posUp, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "moveL回到上方失败" << endl;

        result.result = "moveL回到上方失败";
        handle.setAborted(result);

        return;
    }

    // 先MoveJ到桌子高处，以避免碰撞
    double agvAboveAngles[6] = {
            -9.263 * DE2RA,
            12.880 * DE2RA,
            -35.598 * DE2RA,
            34.058 * DE2RA,
            -90.864 * DE2RA,
            -66.888 * DE2RA
    };
    rst = Robot::getInstance()->moveJ(agvAboveAngles, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "MoveJ到桌子高处，以避免碰撞" << endl;
        result.result = "MoveJ到桌子高处，以避免碰撞";
        handle.setAborted(result);
        return;
    }

    // 移动到固定的目标位置放置
    std::vector<int> targetCenter = {1224, 775};
    std::vector<int> targetVectorX = {100, 0};
    // 构建目标位置位姿
    Mat target2camera = getBoxMat(targetCenter, targetVectorX, 950);

    Mat targetMatUp = exMat * target2camera * toolMatUpInv;
    double *targetPoseUp = convert2pose(targetMatUp);

    // MoveJ到目标位置（目标位置上方）
    rst = Robot::getInstance()->moveJwithPose(targetPoseUp, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "MoveJ到目标位置上方失败" << endl;
        result.result = "MoveJ到目标位置上方失败";
        handle.setAborted(result);
        return;
    }

    // MoveL下降到目标位置
    Mat targetMat = exMat * target2camera * toolMatInv;
    double *targetPose = convert2pose(targetMat);
    rst = Robot::getInstance()->moveJwithPose(targetPose, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "MoveL下降到目标位置失败" << endl;
        result.result = "MoveL下降到目标位置失败";
        handle.setAborted(result);
        return;
    }

    // 打开夹爪，放下盒子
    open_gripper();

    // MoveL上升到目标位置上方
    rst = Robot::getInstance()->moveJwithPose(targetPoseUp, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "MoveL下降到目标位置失败" << endl;
        result.result = "MoveL下降到目标位置失败";
        handle.setAborted(result);
        return;
    }

    // MoveJ回到待命位置
    double defaultAngles[6] = {
            0.516 * DE2RA,
            -25.956 * DE2RA,
            -75.008 * DE2RA,
            38.053 * DE2RA,
            -92.830 * DE2RA,
            0.447 * DE2RA
    };
    rst = Robot::getInstance()->moveJ(defaultAngles, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "回到待命位置失败" << endl;
        result.result = "回到待命位置失败";
        handle.setAborted(result);
        return;
    }

    result.result = "成功!";
    handle.setSucceeded();
}
// line_poses: type 0是原料 1是成品
//  agv_poses: type 0是原料 1是成品

void do_blanking(ServerGoalHandle &handle) {
    // 下料
    //1. 获取图片，并解析图片，获得位置和姿态信息
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<itheima_msgs::GetBoxPoses>("/box/poses");
    client.waitForExistence();
    itheima_msgs::GetBoxPoses service;
    client.call(service);
    auto line_poses = service.response.line_poses;
    client.shutdown();

    std::cout << "-----------------------------1" << std::endl;
    if (line_poses.empty()) {
        // 没有盒子
        itheima_msgs::ArmWorkResult result;
        result.result = "No box checked";
        handle.setRejected(result);
        return;
    }

    std::cout << "-----------------------------2" << std::endl;
    auto first_pose = line_poses[0];
    if (first_pose.type != 1) {
        // 有盒子但是最左边的不是产品区的盒子
        itheima_msgs::ArmWorkResult result;
        result.result = "No product box checked";
        handle.setRejected(result);
        return;
    }

    std::cout << "-----------------------------3" << std::endl;
    // 激活
    handle.setAccepted();
    itheima_msgs::ArmWorkResult result;

    std::vector<int> center = first_pose.center;
    std::vector<int> vect = first_pose.vect;

//    vector<int> center {1244, 775};
//    vector<int> vect_x {60 ,-82};
    //2. 机械臂做相应的操作
    // 取出外参Tc，转成4x4齐次矩阵 ---------------------------------------- ③

    // 取出盒子位姿T1，转成4x4齐次矩阵 ------------------------------------- ②
    // 用二维坐标构建三维位姿
    // [463, 201], [ -14, -100]
    Mat box2CameraMat = getBoxMat(center, vect, 950);

    const Mat &toolMatInv = homogeneousInverse(toolMat);
    cout << "exMat:\n" << exMat << endl;
    cout << "box2CameraMat:\n" << box2CameraMat << endl;
    // 测试模板抓取  `⑤ = ③ · ② · inv(④)`
    Mat_<double> finalMat = exMat * box2CameraMat * toolMatInv;

    double *pos = convert2pose(finalMat);

    // ----------------------------------定义z后退12cm的位置
    Mat_<double> toolMatUp;
    toolMat.copyTo(toolMatUp);
    toolMatUp.at<double>(2, 3) += 0.12f; // z增加12cm
    const Mat &toolMatUpInv = homogeneousInverse(toolMatUp);
    Mat_<double> finalMatUp = exMat * box2CameraMat * toolMatUpInv;
    double *posUp = convert2pose(finalMatUp);

    cout << "x: " << pos[0] << " y: " << pos[1] << " z: " << pos[2];
    cout << " r: " << pos[3] * RA2DE << " p: " << pos[4] * RA2DE << " y: " << pos[5] * RA2DE << endl;

//    if (true) { return; }

    aubo_robot_namespace::JointParam jointParam;
    Robot::getInstance()->robotServiceGetJointAngleInfo(jointParam);

    // 先MoveJ移动到上方12cm位置
    int rst = Robot::getInstance()->moveJwithPose(posUp, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "移动到上方失败" << endl;

        result.result = "移动到上方失败";
        handle.setAborted(result);

        return;
    }
    // 打开夹爪
    open_gripper();

    // 再MoveL下降放到正确位置（抓取）
    rst = Robot::getInstance()->moveL(pos, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "moveL下降到位失败" << endl;

        result.result = "moveL下降到位失败";
        handle.setAborted(result);

        return;
    }

    // 闭合夹爪，夹取盒子
    close_gripper();

    // 后MoveL上升到12cm位置
    rst = Robot::getInstance()->moveL(posUp, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "moveL回到上方失败" << endl;

        result.result = "moveL回到上方失败";
        handle.setAborted(result);

        return;
    }

    // TODO: 判断成品目标位置是否是空着的，非空的话，换个位置

    std::vector<int> targetCenter = {460, 180};
    std::vector<int> targetVectorX = {100, 0};
    // 构建目标位置位姿
    Mat target2camera = getBoxMat(targetCenter, targetVectorX, 1095);

    Mat targetMatUp = exMat * target2camera * toolMatUpInv;
    double *targetPoseUp = convert2pose(targetMatUp);

    // 先MoveJ到桌子高处，以避免碰撞
    double agvAboveAngles[6] = {
             -9.263 * DE2RA,
             12.880 * DE2RA,
            -35.598 * DE2RA,
             34.058 * DE2RA,
            -90.864 * DE2RA,
            -66.888 * DE2RA
    };
    rst = Robot::getInstance()->moveJ(agvAboveAngles, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "MoveJ到桌子高处，以避免碰撞" << endl;
        result.result = "MoveJ到桌子高处，以避免碰撞";
        handle.setAborted(result);
        return;
    }

    // MoveJ到目标位置（目标位置上方）
    rst = Robot::getInstance()->moveJwithPose(targetPoseUp, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "MoveJ到目标位置上方失败" << endl;
        result.result = "MoveJ到目标位置上方失败";
        handle.setAborted(result);
        return;
    }

    // MoveL下降到目标位置
    Mat targetMat = exMat * target2camera * toolMatInv;
    double *targetPose = convert2pose(targetMat);
    rst = Robot::getInstance()->moveJwithPose(targetPose, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "MoveL下降到目标位置失败" << endl;
        result.result = "MoveL下降到目标位置失败";
        handle.setAborted(result);
        return;
    }

    // 打开夹爪，放下盒子
    open_gripper();

    // MoveL上升到目标位置上方
    rst = Robot::getInstance()->moveJwithPose(targetPoseUp, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "MoveL下降到目标位置失败" << endl;
        result.result = "MoveL下降到目标位置失败";
        handle.setAborted(result);
        return;
    }

    // MoveJ回到待命位置
    double defaultAngles[6] = {
                0.516 * DE2RA,
              -25.956 * DE2RA,
              -75.008 * DE2RA,
               38.053 * DE2RA,
              -92.830 * DE2RA,
                0.447 * DE2RA
    };
    rst = Robot::getInstance()->moveJ(defaultAngles, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "回到待命位置失败" << endl;
        result.result = "回到待命位置失败";
        handle.setAborted(result);
        return;
    }

    result.result = "成功!";
    handle.setSucceeded();
}

void do_goal(ServerGoalHandle &handle) {
    auto id = handle.getGoalID().id;

    itheima_msgs::ArmWorkResult result;
    if (canceledIds.count(id)) {
        result.result = "canceled";
        handle.setCanceled(result);
        return;
    }

    auto goal = handle.getGoal();
    if (goal->type == 0) {
        // 上料
        do_feeding(handle);
    } else {
        // 下料
        do_blanking(handle);
    }
}

void do_cb() {
    while (ros::ok()) {
        ServerGoalHandle handle;
        try {
            handle = handleQueue.take();
        } catch (exception e) {
            continue;
        }
        do_goal(handle);
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "aubo_blanking_node");
    ros::NodeHandle node("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    string host = node.param<string>("aubo_host", "192.168.1.101");
    int port = node.param<int>("aubo_port", 8899);

    string pkg_path = node.param<string>("aubo_ctrl_pkg_path", "");
    // 内参文件路径
    const cv::String inCailFilePath = pkg_path + "/assets/calibration_in_params.yml";
// 外参文件路径
    const cv::String exCailFilePath = pkg_path + "/assets/calibration_ex_params_mi.yml";

    std::cout << ">>>> inCailFilePath: " << inCailFilePath << std::endl;
    std::cout << ">>>> exCailFilePath: " << exCailFilePath << std::endl;

    exMat = getExMat(exCailFilePath);

    FileStorage fs(inCailFilePath, FileStorage::READ);
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();

//    char *ip = new char[host.size()];
//    memcpy(&ip[0], &host[0], host.size());

    const char *ip = host.c_str();
    int ret = Robot::getInstance()->connect(ip, port);
    if (ret != 0) {
        ROS_ERROR_STREAM("aubo connect failed");
        return -1;
    }
    Robot::getInstance()->setOffset(3.0f / 1000.0f, -0.0f / 1000.0f, -0.0f / 1000.0f);

    string actionName = "/aubo/ctrl";
    ActionServer server(node, actionName,
                        boost::bind(&goal_cb, _1),
                        boost::bind(&cancel_cb, _1),
                        false);
    server.start();

    // 开启处理任务的线程
    new thread(do_cb);

    std::cout << "-------------------------------aubo ctrl successful" << std::endl;

    ros::waitForShutdown();
    Robot::getInstance()->disConnect();
    handleQueue.release();

    return 0;
}
