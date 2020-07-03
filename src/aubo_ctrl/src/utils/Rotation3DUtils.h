#ifndef STUDY_ROBOT_ROTATION_UTILS_H
#define STUDY_ROBOT_ROTATION_UTILS_H

#include <iostream>
#include <opencv/cv.hpp>
#include <Eigen/Geometry>
//#include <Eigen/Core>
//// 稠密矩阵的代数运算（逆，特征值等）
//#include <Eigen/Dense>

using namespace std;
using namespace cv;


#define DE2RA M_PI / 180
#define RA2DE 180 / M_PI


/**
 * 给定x方向vec，参考方向rVec，确定一个坐标系。
 * 1. 根据x轴向量vec和参考向量rVec  叉乘（cross）得到z轴向量norVec（即法向量，垂直于vec和rVec所在平面）
 * 2. 根据x轴向量vec和z轴向量norVec 叉乘（cross）得到y轴向量yVec
 * 3. 通过x、y、z三个轴的向量构建一个旋转矩阵
 * 4. 将旋转矩阵转成PRY
 *
 * @param norm
 * @return
 */
vector<double> calcRPY(vector<double> vec, vector<double> rVec) {
    // 1.x方向, 并归一化
    Eigen::Vector3d xVec(vec[0], vec[1], vec[2]);
    xVec = xVec.normalized();

    // 2.给定参考方向，此方向根据实际情况指定
    Eigen::Vector3d refVec(rVec[0], rVec[1], rVec[2]);

    // 3.通过叉乘，计算normal向量作为z轴
    Eigen::Vector3d norVec = xVec.cross(refVec);
    norVec = norVec.normalized();

    /// 4.通过叉乘，计算y轴向量
    Eigen::Vector3d yVec = norVec.cross(xVec);
    yVec = yVec.normalized();

    /// 5.根据三个坐标轴，可以构建一个旋转矩阵，
    /// 输入依次是
    /// x轴向量
    /// y轴向量
    /// z轴向量(法向量)
    std::cout << "xVec:\t" << xVec << std::endl;
    std::cout << "yVec:\t" << yVec << std::endl;
    std::cout << "norVec:\t" << norVec << std::endl;

    Eigen::Matrix3d Rot;
    Rot << xVec[0], yVec[0], norVec[0],
            xVec[1], yVec[1], norVec[1],
            xVec[2], yVec[2], norVec[2];

    ///最后，需要注意的是，此处的RPY，是绕动坐标系变换的，顺序是z,y,x
    ///而UR驱动中,moveL的输入里，RPY是按x,y,z顺序输入的，
    ///因此，实际使用时，需要逆转输出
    Eigen::Vector3d euler_angles = Rot.eulerAngles(2, 1, 0);
    cout << "RotationMatrix2euler result is:" << endl;
    cout << "yaw(z) pitch(y) roll(x) = " << euler_angles.transpose() << endl;
    std::vector<double> result{euler_angles[0], euler_angles[1], euler_angles[2]};
//    result = normalize(rpy);
    return result;

};

/**
 * 将2D坐标（像素）转成3D坐标（米）
 * @param depthMat  深度矩阵
 * @param points    图像点列表
 * @param cameraMatrix  相机内参
 * @param points3D      输出3D点
 * @return
 */
void getXYZ(Mat &depthMat,
                       vector<Point2i> &points,
                       Mat &cameraMatrix,
                       vector<Point3d> &points3D) {

    double camera_fx = cameraMatrix.at<double>(0, 0);
    double camera_fy = cameraMatrix.at<double>(1, 1);
    double camera_cx = cameraMatrix.at<double>(0, 2);
    double camera_cy = cameraMatrix.at<double>(1, 2);

    unsigned long size = points.size();
    for (int i = 0; i < size; ++i) {
        Point_<int> &point = points[i];
        Point3d cameraPos;
        // 列
        int u = point.x;
        // 行
        int v = point.y;

        ushort depth = depthMat.ptr<ushort>(v)[u];

        float d = depth * 1.0;
        // 内参数据
        // 计算相机坐标系中的值
        double z = double(d) / 1000.0; //单位是米

        if (!isnan(z) && abs(z) >= 0.0001) {
            double x = (u - camera_cx) * z / camera_fx;
            double y = (v - camera_cy) * z / camera_fy;

            cameraPos.x = x;
            cameraPos.y = y;
            cameraPos.z = z;
            points3D.push_back(cameraPos);
        }

    }

}

/**
 * 检查是否是旋转矩阵
 *
 * 齐次矩阵
 * 矩阵的转置T 等于 矩阵的逆inv
**/
bool isRotationMatrix(Mat &R) {
    Mat Rt;
    transpose(R, Rt);
    // 应该得到单位矩阵
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3, 3, shouldBeIdentity.type());

    return norm(I, shouldBeIdentity) < 1e-6;
}

/**
 * 通过给定的旋转矩阵计算对应的欧拉角
**/
Vec3f rotationMatrixToEulerAngles(Mat &R) {
    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular) {
        x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2(-R.at<double>(2, 0), sy);
        z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);
}

/**
 * 求齐次矩阵的逆矩阵
 * @param T
 * @return
 */
static Mat homogeneousInverse(const Mat &T) {
    CV_Assert(T.rows == 4 && T.cols == 4);

    Mat R = T(Rect(0, 0, 3, 3));
    Mat t = T(Rect(3, 0, 1, 3));
    Mat Rt = R.t();
    Mat tinv = -Rt * t;

    Mat Tinv = Mat::eye(4, 4, T.type());
    Rt.copyTo(Tinv(Rect(0, 0, 3, 3)));
    tinv.copyTo(Tinv(Rect(3, 0, 1, 3)));

    return Tinv;
}

/**
 * 将旋转矩阵和平移向量转成4x4的齐次变换矩阵
 * @param rotationMat    旋转矩阵
 * @param translationMat 平移向量
 * @return 齐次变换矩阵
 */
static Mat toHomogeneousMat(const Mat &rotationMat, const Mat &translationMat) {
    Mat m = Mat::eye(4, 4, CV_64FC1); // rotationMat.type()

    Mat R = m(Rect(0, 0, 3, 3));
//    translateMat.copyTo(m(Rect(3, 0, 1, 3)));
    rotationMat.convertTo(R, CV_64F);
    Mat t = m(Rect(3, 0, 1, 3));
    translationMat.convertTo(t, CV_64F);
    return m;
};

/**
 * 取出齐次变换矩阵的旋转矩阵和平移向量
 * @param T
 * @param outputRotation
 * @param outputTranslation
 */
static void splitHomogeneousMat(Mat &T, Mat &outputRotation, Mat &outputTranslation) {
    CV_Assert(T.rows == 4 && T.cols == 4);

    Mat R = T(Rect(0, 0, 3, 3));
    Mat t = T(Rect(3, 0, 1, 3));

    outputRotation = R;
    outputTranslation = t;
};

/**
 * 将4x4变换矩阵转成xyzrpy位姿
 * @param finalMat
 * @return
 */
double *convert2pose(Mat &finalMat) {
    Mat R = Mat::eye(3, 3, CV_64FC1);
    Mat t = Mat::zeros(3, 1, CV_64FC1);

    // 将4x4变换矩阵，拆分成3x3旋转矩阵 + 3x1平移向量
    splitHomogeneousMat(finalMat, R, t);

    // 转成xyzrpy
    const Vec3f &vec = rotationMatrixToEulerAngles(R);
    double* pos =  new double[6]{t.at<double>(0), t.at<double>(1), t.at<double>(2), vec[0], vec[1], vec[2]};
    return pos;
}

/**
欧拉角计算对应的旋转矩阵
**/
Mat eulerAnglesToRotationMatrix(Vec3f &theta) {
    // 计算旋转矩阵的X分量
    Mat R_x = (Mat_<double>(3, 3) <<
                                  1, 0, 0,
            0, cos(theta[0]), -sin(theta[0]),
            0, sin(theta[0]), cos(theta[0])
    );

    // 计算旋转矩阵的Y分量
    Mat R_y = (Mat_<double>(3, 3) <<
                                  cos(theta[1]), 0, sin(theta[1]),
            0, 1, 0,
            -sin(theta[1]), 0, cos(theta[1])
    );

    // 计算旋转矩阵的Z分量
    Mat R_z = (Mat_<double>(3, 3) <<
                                  cos(theta[2]), -sin(theta[2]), 0,
            sin(theta[2]), cos(theta[2]), 0,
            0, 0, 1
    );

    // 合并
    Mat R = R_z * R_y * R_x;
    return R;
}


#endif //STUDY_ROBOT_ROTATION_UTILS_H