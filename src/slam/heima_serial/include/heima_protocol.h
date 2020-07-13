//
// Created by Kaijun on 2020/5/30.
//

#ifndef ROSSERIAL_DEMO_USER_PROTOCOL_H
#define ROSSERIAL_DEMO_USER_PROTOCOL_H

#include <stdio.h>

enum _ptl_type
{
    VAL_START = 0,
    VAL_POSE,
    VAL_VEL,
    VAL_PID,
    VAL_IMU,
    VAL_END
};
#define _SERIAL_SYN_CODE_START 0xFA
// 1 字节对齐
#pragma pack(1)
typedef struct _serial_data
{
    uint8_t syn;
    uint8_t type;
    struct {
        struct{
            float liner[3];
            float angular[3];
        }vel;
        struct{
            bool rot_ok,acc_ok,mag_ok;
            double rot[3],acc[3],mag[3];
        }imu;
        float pid[3];
    }dat;
    uint8_t syn_CR;
    uint8_t syn_LF;
}serialData;
#pragma pack();


// 定义接收数据帧格式
#define SERIAL_SIGN_FLAG1  0xff
#define SERIAL_SIGN_FLAG2  0xfe
#pragma pack(1)
typedef struct
{
    double x_vel;
    double y_vel;
    double z_vel;

    double x_acc;
    double y_acc;
    double z_acc;

    double x_gyro;
    double y_gyro;
    double z_gyro;

}SerialRxData;
#pragma pack();


#pragma pack(1)
typedef struct
{
    uint8_t syn1;
    uint8_t syn2;
    uint8_t a_vel;
    uint8_t b_vel;
    uint8_t a_direct;
    uint8_t b_direct;
    uint8_t flag0 = 0;
    uint8_t flag1 = 0;
    uint8_t flag2 = 0;
    uint8_t flag3 = 0;
}SerialTxData;
#pragma pack();



#endif //ROSSERIAL_DEMO_USER_PROTOCOL_H
