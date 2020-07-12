//
// Created by Kaijun on 2020/6/21.
//

#ifndef HEIMA3ROBOT_KINEMATICS_H
#define HEIMA3ROBOT_KINEMATICS_H

#include <stdint.h>

class Kinematics {

public:
    struct RPM{
        int motorLeftRpm;
        int leftSignalNum;
        uint8_t leftSignalDirect;
        int motorRightRpm;
        int rightSignalNum;
        uint8_t rightSignalDirect;
    };

    struct Velocity{
        double linear_velocity;
        double angular_velocity;
    };

private:
    double wheel_diameter;
    double half_width;
    double wheel_circle;

public:
    /**
     * 根据线速度和角速度计算左右轮子的转速
     * @param wheel_diameter  轮子的直径
     * @param half_width   小车左右轮子中心距离的一半
     */

    Kinematics(double wheel_diameter,double half_width);
    Velocity getVelocityByNum(double l_num,double r_num);
    RPM calculateRPM(double linear_x,double angular_z);
    Velocity getVelocity(double rpm_l,double rmp_r);

};


#endif //HEIMA3ROBOT_KINEMATICS_H
