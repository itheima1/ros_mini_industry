//
// Created by Kaijun on 2020/6/21.
//

#include "Kinematics.h"
#include <math.h>
#include <ros/ros.h>

/**
 * @param wheel_diameter 直径
 * @param half_width  小车宽度的一半
 */
Kinematics::Kinematics(double wheel_diameter,double half_width){
    this->wheel_diameter = wheel_diameter;
    this->half_width = half_width;
    this->wheel_circle = 3.1415926*wheel_diameter;
}

uint8_t getDirect(double num){
    if(num > 0){
        return 1;
    }else{
        return 0;
    }
}

/**
 *
 * @param linear_x  上位机发送过来的线速度
 * @param angular_z  上位机发送过来的角速度
 * @return
 */
Kinematics::RPM Kinematics::calculateRPM(double linear_x,double angular_z){
    double v_l = linear_x - angular_z*this->half_width; // m/s
    double v_r = linear_x + angular_z*this->half_width;

    //  m/s ----> rpm : 圈/分钟
    /**
     *  圈/分钟 ---> 米/s
     *  110圈/分钟  ---> PI*直径 * 110 / 分钟 --> PI*直径 *圈数/60s
     *
     *  米/s --> 60 * 米/min --> 60*米/min/PI*直径/min
     */
    double v_l_rpm = v_l*60/this->wheel_circle;
    double v_r_rpm = v_r*60/this->wheel_circle;


    int l_signal_num = abs(v_l_rpm) * 11750 / 6000;
    uint8_t l_direct = getDirect(v_l_rpm);
    int r_signal_num = abs(v_r_rpm) * 11750 / 6000;
    uint8_t r_direct = getDirect(v_r_rpm);



    Kinematics::RPM rpm_result;

    rpm_result.motorLeftRpm = v_l_rpm;
    rpm_result.motorRightRpm = v_r_rpm;

    rpm_result.leftSignalNum = l_signal_num;
    rpm_result.leftSignalDirect = l_direct;
    rpm_result.rightSignalNum = r_signal_num;
    rpm_result.rightSignalDirect = r_direct;
    return rpm_result;
}


/*
 * 根据左右轮子的信号数量 获取小车当前的线速度 和 角速度
 * */
Kinematics::Velocity Kinematics::getVelocityByNum(double l_num,double r_num){

    double rpm_l = l_num*100*60/11750;
    double rpm_r = r_num*100*60/11750;


    return getVelocity(rpm_l,rpm_r);
}

/*
 * 根据左右轮子的rpm 获取小车当前的线速度 和 角速度
 * */
Kinematics::Velocity Kinematics::getVelocity(double rpm_l,double rpm_r){
    // 求线速度 圈/分钟
    double linear_velocity = (rpm_r  + rpm_l)/2;
    // 求角速度
    double angular_velocity = (rpm_r - rpm_l)/(2*half_width);

    // 将单位转 m/s  弧度/s   圈/分钟 --> 圈/s --> 圈*周长m   圈/分钟 ===> 圈*周长/60
    double linear_velocity_s = linear_velocity*wheel_circle/60;

    // 角速度
    double angular_velocity_s = angular_velocity*wheel_circle/60;

    Velocity velocity;
    velocity.linear_velocity = linear_velocity_s;
    velocity.angular_velocity = angular_velocity_s;
    return velocity;
}