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

float z_up = 0.12f; // 夹取抬起高度 12cm
float catch_depth = 10.0f; // 抓取的深度
float box_height = 45.0f; // 盒子的高度
float line_box_distance_z ; // 相机距离传送带盒子表面距离
float avg_box_distance_z  ;  // 相机距离AGV车盒子表面距离

// 待命位置
double defaultAngles[6] = {
          0.000 * DE2RA,
        -43.000 * DE2RA,
        -90.000 * DE2RA,
         30.000 * DE2RA,
        -90.000 * DE2RA,
          0.000 * DE2RA
};
// 小车高处安全位置
double agvAboveAngles[6] = {
         35.109 * DE2RA,
        -11.845 * DE2RA,
        -65.565 * DE2RA,
         30.147 * DE2RA,
        -90.770 * DE2RA,
        -66.890 * DE2RA
};

// 传送带原料高处安全位置
double lineRawAboveAngles[6] = {
        47.173 * DE2RA,
        -7.655 * DE2RA,
        -64.068 * DE2RA,
        13.950 * DE2RA,
        -92.124 * DE2RA,
        0.446 * DE2RA
};

static Mat_<double> toolMat;


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
//    usleep(2 * 1000 * 1000);
//    return true;
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

Mat getBoxMat(const vector<int> &center, const vector<int> &vect, double z_distance, bool isFromVectorY = true) {// 将二维像素坐标转成三维坐标
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

void do_feeding(ServerGoalHandle &handle) {
    // 上料
    //1. 获取图片，并解析图片，获得位置和姿态信息
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<itheima_msgs::GetBoxPoses>("/box/poses");
    client.waitForExistence();
    itheima_msgs::GetBoxPoses service;
    client.call(service);
    auto agv_poses = service.response.agv_poses;
    auto line_poses = service.response.line_poses;
    client.shutdown();

    if (agv_poses.empty()) {
        // 没有盒子
        itheima_msgs::ArmWorkResult result;
        result.result = "No box checked";
        handle.setRejected(result);
        return;
    }

//    itheima_msgs::BoxPosePtr boxPose = nullptr;
    bool hasRaw = false;
    itheima_msgs::BoxPose rawPose;
    for (itheima_msgs::BoxPose pose : agv_poses) {
        if (pose.type == 0) { // 原料区的盒子
            rawPose = pose;
            hasRaw = true;
            break;
//            boxPose = itheima_msgs::BoxPosePtr(&pose);
        }
    }

    if (!hasRaw) {
        // 有盒子但是不是原料区的盒子
        itheima_msgs::ArmWorkResult result;
        result.result = "No source box checked";
        handle.setRejected(result);
        return;
    }


    // 先准备好下料的传送带上的目标空白放置区域 type == 2
    auto target_pose = line_poses[line_poses.size() - 1];
    if (target_pose.type != 2) {
        // 最右边的不是Line上的空白目标位置
        itheima_msgs::ArmWorkResult result;
        result.result = "No blank target checked";
        handle.setRejected(result);
        return;
    }

    std::vector<int> targetCenter = target_pose.center;
    std::vector<int> targetVectorY = target_pose.vect;

    // 激活
    handle.setAccepted();
    itheima_msgs::ArmWorkResult result;

    std::vector<int> center = rawPose.center;
    std::vector<int> vect = rawPose.vect;

    // 取出盒子位姿T1，转成4x4齐次矩阵 ------------------------------------- ②
    // 上料目标 （传送带区域）用二维坐标构建三维位姿
    Mat box2CameraMat = getBoxMat(center, vect, avg_box_distance_z);

    const Mat &toolMatInv = homogeneousInverse(toolMat);
    cout << "exMat:\n" << exMat << endl;
    cout << "box2CameraMat:\n" << box2CameraMat << endl;
    // 测试模板抓取  `⑤ = ③ · ② · inv(④)`
    Mat_<double> finalMat = exMat * box2CameraMat * toolMatInv;

    double *pos = convert2pose(finalMat);

    // ----------------------------------定义z后退12cm的位置
    Mat_<double> toolMatUp;
    toolMat.copyTo(toolMatUp);
    toolMatUp.at<double>(2, 3) += z_up; // z增加12cm
    const Mat &toolMatUpInv = homogeneousInverse(toolMatUp);
    Mat_<double> finalMatUp = exMat * box2CameraMat * toolMatUpInv;
    double *posUp = convert2pose(finalMatUp);

    cout << "x: " << pos[0] << " y: " << pos[1] << " z: " << pos[2];
    cout << " r: " << pos[3] * RA2DE << " p: " << pos[4] * RA2DE << " y: " << pos[5] * RA2DE << endl;

//    if (true) { return; }

    aubo_robot_namespace::JointParam jointParam;
    Robot::getInstance()->robotServiceGetJointAngleInfo(jointParam);


    int rst = Robot::getInstance()->moveJ(defaultAngles, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "回到待命位置失败"+ to_string(rst) << endl;
        result.result = "回到待命位置失败"+ to_string(rst);
        handle.setAborted(result);
        return;
    }

    // 先MoveJ移动到上方12cm位置
    rst = Robot::getInstance()->moveJwithPose(posUp, true);
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
    rst = Robot::getInstance()->moveJ(agvAboveAngles, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "MoveJ到桌子高处，以避免碰撞"+ to_string(rst) << endl;
        result.result = "MoveJ到桌子高处，以避免碰撞"+ to_string(rst);
        handle.setAborted(result);
        return;
    }

    // 移动到固定的目标位置放置
//    std::vector<int> targetCenter = {1224, 775};
//    std::vector<int> targetVectorX = {100, 0};

    // 再MoveJ到传送带上方的安全位置， 以避免碰撞
    rst = Robot::getInstance()->moveJ(lineRawAboveAngles, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "MoveJ到传送带高处，以避免碰撞"+ to_string(rst) << endl;
        result.result = "MoveJ到传送带高处，以避免碰撞"+ to_string(rst);
        handle.setAborted(result);
        return;
    }

    // 构建目标位置位姿
    Mat target2camera = getBoxMat(targetCenter, targetVectorY, line_box_distance_z);
    Mat targetMatUp = exMat * target2camera * toolMatUpInv;
    double *targetPoseUp = convert2pose(targetMatUp);

    // MoveJ到目标位置（目标位置上方）
    rst = Robot::getInstance()->moveJwithPose(targetPoseUp, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "MoveJ到目标位置上方失败" + to_string(rst) << endl;
        result.result = "MoveJ到目标位置上方失败" + to_string(rst);
        handle.setAborted(result);
        return;
    }

    // MoveL下降到目标位置
    Mat targetMat = exMat * target2camera * toolMatInv;
    double *targetPose = convert2pose(targetMat);
    rst = Robot::getInstance()->moveL(targetPose, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "MoveL下降到目标位置失败" << endl;
        result.result = "MoveL下降到目标位置失败"+ to_string(rst);
        handle.setAborted(result);
        return;
    }

    // 打开夹爪，放下盒子
    open_gripper();

    // MoveL上升到目标位置上方
    rst = Robot::getInstance()->moveL(targetPoseUp, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "MoveL上升到目标位置上方"+ to_string(rst) << endl;
        result.result = "MoveL上升到目标位置上方失败"+ to_string(rst);
        handle.setAborted(result);
        return;
    }

    // 再MoveJ到传送带上方的安全位置， 以避免碰撞
    rst = Robot::getInstance()->moveJ(lineRawAboveAngles, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "MoveJ到传送带高处，以避免碰撞"+ to_string(rst) << endl;
        result.result = "MoveJ到传送带高处，以避免碰撞"+ to_string(rst);
        handle.setAborted(result);
        return;
    }

    // MoveJ回到待命位置
    rst = Robot::getInstance()->moveJ(defaultAngles, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "回到待命位置失败" << endl;
        result.result = "回到待命位置失败";
        handle.setAborted(result);
        return;
    }

    result.result = "成功!";
    handle.setSucceeded(result);
}
// line_poses: type 0是原料 1是成品
//  agv_poses: type 0是原料 1是成品

void do_blanking(ServerGoalHandle &handle) {
    // 下料 ====================================================================================================
    //1. 获取图片，并解析图片，获得位置和姿态信息
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<itheima_msgs::GetBoxPoses>("/box/poses");
    client.waitForExistence();
    itheima_msgs::GetBoxPoses service;
    client.call(service);
    auto line_poses = service.response.line_poses;
    auto agv_poses = service.response.agv_poses;
    client.shutdown();

    if (line_poses.empty()) {
        // 没有盒子
        itheima_msgs::ArmWorkResult result;
        result.result = "No box checked";
        handle.setRejected(result);
        return;
    }

//    itheima_msgs::BoxPosePtr boxPose = nullptr;
    bool hasPro = false;
    itheima_msgs::BoxPose proPose;
    for (itheima_msgs::BoxPose pose : line_poses) {
        if (pose.type == 1) { // 取最右侧的产品区的盒子
            proPose = pose;
            hasPro = true;
//            boxPose = itheima_msgs::BoxPosePtr(&pose);
        }
    }
    if (!hasPro) {
        // 有盒子，但是产品区没有盒子
        itheima_msgs::ArmWorkResult result;
        result.result = "No product box on line checked";
        handle.setRejected(result);
        return;
    }

    // 提前先把放置位置确认好
    Mat_<double> toolMatUp;
    toolMat.copyTo(toolMatUp);
    toolMatUp.at<double>(2, 3) += z_up; // z增加12cm
    const Mat &toolMatUpInv = homogeneousInverse(toolMatUp);
    const Mat &toolMatInv = homogeneousInverse(toolMat);

    // TODO: 测试用， 上线记得把 line_poses 改成 agv_poses 先准备好目标区域位置
    // 判断小车AGV产品是否有空白的目标位置，没有的话，提示用户空出目标位置

    // 倒序循环判断有没有可以逆解成功的目标位置
    itheima_msgs::BoxPose *agv_target_pose = nullptr;
    double *targetPose;
    double *targetPoseUp;
    for (int j = agv_poses.size() - 1; j >= 0 ; j--) {
        auto pose = agv_poses[j];
        if (pose.type == 2) {
            agv_target_pose = new itheima_msgs::BoxPose(pose);

            std::vector<int> targetCenter = agv_target_pose->center;
            std::vector<int> targetVectorY = agv_target_pose->vect;

            // 构建agv目标位置位姿
            Mat target2camera = getBoxMat(targetCenter, targetVectorY, avg_box_distance_z);
            Mat targetMatUp = exMat * target2camera * toolMatUpInv;
            targetPoseUp = convert2pose(targetMatUp);

            Mat targetMat = exMat * target2camera * toolMatInv;
            targetPose = convert2pose(targetMat);

            // 如果两个位置都逆解成功，则直接停止循环
            int rst1 = Robot::getInstance()->moveJwithPoseTest(targetPoseUp);
            int rst2 = Robot::getInstance()->moveJwithPoseTest(targetPose);
            if (rst1 == aubo_robot_namespace::ErrnoSucc && rst2 == aubo_robot_namespace::ErrnoSucc) {
                break;
            }
        }
    }
    if (agv_target_pose == nullptr || agv_target_pose->type != 2) {
        // 最后放的不是AGV上产品区的空白目标位置
        itheima_msgs::ArmWorkResult result;
        result.result = "No blank target checked";
        handle.setRejected(result);
        return;
    }
    // 激活
    handle.setAccepted();
    itheima_msgs::ArmWorkResult result;

    std::vector<int> center = proPose.center;
    std::vector<int> vect = proPose.vect;
//    vector<int> center {1244, 775};
//    vector<int> vect_x {60 ,-82};
    //2. 机械臂做相应的操作
    // 取出外参Tc，转成4x4齐次矩阵 ---------------------------------------- ③

    // 取出盒子位姿T1，转成4x4齐次矩阵 ------------------------------------- ②
    // 用二维坐标构建三维位姿
    // [463, 201], [ -14, -100]
    Mat box2CameraMat = getBoxMat(center, vect, line_box_distance_z);

    cout << "exMat:\n" << exMat << endl;
    cout << "box2CameraMat:\n" << box2CameraMat << endl;
    // 测试模板抓取  `⑤ = ③ · ② · inv(④)`
    Mat_<double> finalMat = exMat * box2CameraMat * toolMatInv;

    double *pos = convert2pose(finalMat);

    // ----------------------------------定义z后退12cm的位置
    Mat_<double> finalMatUp = exMat * box2CameraMat * toolMatUpInv;
    double *posUp = convert2pose(finalMatUp);

    cout << "x: " << pos[0] << " y: " << pos[1] << " z: " << pos[2];
    cout << " r: " << pos[3] * RA2DE << " p: " << pos[4] * RA2DE << " y: " << pos[5] * RA2DE << endl;

//    if (true) { return; }

    aubo_robot_namespace::JointParam jointParam;
    Robot::getInstance()->robotServiceGetJointAngleInfo(jointParam);

    int rst = Robot::getInstance()->moveJ(defaultAngles, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "blanking回到待命位置失败"+ to_string(rst) << endl;
        result.result = "blanking回到待命位置失败"+ to_string(rst);
        handle.setAborted(result);
        return;
    }

    // 先MoveJ移动到上方12cm位置
    rst = Robot::getInstance()->moveJwithPose(posUp, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "blanking移动到上方失败" << rst << endl;
        result.result = "blanking移动到上方失败";
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

    // TODO: 上线记得删掉，构建line目标位置位姿
//    Mat target2camera = getBoxMat(targetCenter, targetVectorY, line_box_distance_z);


    // TODO: 上线记得删掉，测试代码，移动到上料区上方
//    rst = Robot::getInstance()->moveJ(lineRawAboveAngles, true);

    //  先MoveJ到桌子高处，以避免碰撞
    rst = Robot::getInstance()->moveJ(agvAboveAngles, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "MoveJ到桌子高处，以避免碰撞失败" + to_string(rst) << endl;
        result.result = "MoveJ到桌子高处，以避免碰撞失败"+ to_string(rst);
        handle.setAborted(result);
        return;
    }

    // MoveJ到目标位置（目标位置上方）
    rst = Robot::getInstance()->moveJwithPose(targetPoseUp, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "MoveJ到目标位置上方失败"+ to_string(rst) << endl;
        result.result = "MoveJ到目标位置上方失败"+ to_string(rst);
        handle.setAborted(result);
        return;
    }

    // MoveL下降到目标位置
    rst = Robot::getInstance()->moveL(targetPose, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "MoveL下降到目标位置失败"+ to_string(rst) << endl;
        result.result = "MoveL下降到目标位置失败"+ to_string(rst);
        handle.setAborted(result);
        return;
    }

    // 打开夹爪，放下盒子
    open_gripper();

    // MoveL上升到目标位置上方
    rst = Robot::getInstance()->moveL(targetPoseUp, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "MoveL下降到目标位置失败"+ to_string(rst) << endl;
        result.result = "MoveL下降到目标位置失败"+ to_string(rst);
        handle.setAborted(result);
        return;
    }

    // MoveJ回到待命位置
    rst = Robot::getInstance()->moveJ(defaultAngles, true);
    if (rst != aubo_robot_namespace::ErrnoSucc) {
        cerr << "回到待命位置失败"+ to_string(rst) << endl;
        result.result = "回到待命位置失败"+ to_string(rst);
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

    double kinect_camera_2_line = node.param<double>("kinect_camera_2_line", 1050.0f);
    double line_2_agv_top = node.param<double>("line_2_agv_top", -10.0f);
    std::cout << ">>>> kinect_camera_2_line: " << kinect_camera_2_line << std::endl;

    // 抓取深度
    catch_depth = node.param<double>("catch_depth", 20.0f);
    // 相机距离传送带盒子表面距离
    line_box_distance_z = kinect_camera_2_line - box_height;
    // 相机距离AGV车盒子表面距离
    avg_box_distance_z = line_box_distance_z + line_2_agv_top;


// 定义工具位姿 ----------------------------------------------------- ④
//    double tool_x = 0, tool_y = 0, tool_z = 90; // 标定枪
    double tool_x = 0, tool_y = 0, tool_z = (190.0f - catch_depth);//大环夹爪
    toolMat= (Mat_<double>(4, 4) <<
            1, 0, 0, tool_x / 1000,
            0, 1, 0, tool_y / 1000,
            0, 0, 1, tool_z / 1000,
            0, 0, 0, 1
    );

    string pkg_path = node.param<string>("aubo_ctrl_pkg_path", "");
    // 内参文件路径
    const cv::String inCailFilePath = pkg_path + "/assets/calibration_in_params_mi.yml";
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
//    Robot::getInstance()->setOffset(0.0f / 1000.0f, -0.0f / 1000.0f, -0.0f / 1000.0f);
//    Robot::getInstance()->setOffset(0.0f / 1000.0f, 0.0f / 1000.0f, 0.0f / 1000.0f);
    Robot::getInstance()->setOffset(0.0f / 1000.0f, 0.0f / 1000.0f, 0.0f / 1000.0f);

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
