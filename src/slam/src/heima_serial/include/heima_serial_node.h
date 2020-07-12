//
// Created by itcast on 7/6/20.
//

#ifndef HEIMA_SERIAL_HEIMA_SERIAL_NODE_H
#define HEIMA_SERIAL_HEIMA_SERIAL_NODE_H

using namespace std;
#define SEND_DATA_CHECK   1     //标志位，发送端做校验位
#define READ_DATA_CHECK   0     //标志位，接收端做校验位
#define FRAME_HEADER      0X7B  //帧头，和下位机一致
#define FRAME_TAIL  0X7D //帧尾
#define RECEIVE_DATA_SIZE		24//下位机发过来的数据的长度
#define SEND_DATA_SIZE			11//ROS发给下位机的数据的长度 考虑到时效应短尽短
#define PI 				3.1415926f//圆周率
//这个和陀螺仪设置的量程有关的 转化为度每秒是/65.5 转为弧度每秒/57.3 其子65.5看MPU6050手册，STM32底层FS_SEL=1
#define GYROSCOPE_RATIO	0.00026644f	// 1/65.5/57.30=0.00026644 陀螺仪原始数据换成弧度单位
//这个和陀螺仪设置的量程有关的 转化为度每秒是/65.5 转为弧度每秒/57.3 其子65.5看MPU6050手册，STM32底层FS_SEL=1
#define ACCEl_RATIO 	16384.0f  	// 量程±2g，重力加速度定义为1g等于9.8米每平方秒。
extern sensor_msgs::Imu Mpu6050;

// 速度/位置结构体
typedef struct __Vel_Pos_Data_
{
    float X;
    float Y;
    float Z;
}Vel_Pos_Data;


typedef struct __MPU6050_DATA_
{
    short accele_x_data;
    short accele_y_data;
    short accele_z_data;
    short gyros_x_data;
    short gyros_y_data;
    short gyros_z_data;

}MPU6050_DATA;

//DATE：2020-5-31

typedef struct _SEND_DATA_
{
    uint8_t tx[SEND_DATA_SIZE];
    float X_speed;
    float Y_speed;
    float Z_speed;
    unsigned char Frame_Tail;    //1个字节  帧尾 校验位

}SEND_DATA;
//DATE：2020-5-31

typedef struct _RECEIVE_DATA_
{
    uint8_t rx[RECEIVE_DATA_SIZE];
    uint8_t Flag_Stop;
    unsigned char Frame_Header; //1个字节 帧头
    float X_speed;
    float Y_speed;
    float Z_speed;
    float Power_Voltage;
    unsigned char Frame_Tail;//1个字节  帧尾 校验位
}RECEIVE_DATA;





#endif //HEIMA_SERIAL_HEIMA_SERIAL_NODE_H
