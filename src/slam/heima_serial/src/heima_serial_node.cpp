//
// Created by itcast on 5/29/20.
//
#include <ros/ros.h>
#include <string.h>
#include <std_msgs/String.h>
#include <sstream>

#include "heima_protocol.h"

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point32.h>
#include "heima_serial_node.h"
#include <sensor_msgs/Imu.h>
#include <heima_protocol.h>
#include <serial/serial.h>
#include <heima_msgs/Imu.h>
#include <heima_msgs/Velocities.h>
#include "Kinematics.h"

bool param_use_debug_imu;
bool param_use_debug_cmd;
bool param_use_debug_vel;
bool debug = true;

std::string param_port_path;
int param_baudrate_;

Kinematics kinematics(0.21,0.29);

serial::Serial ros_ser;

int getDirect(uint8_t flag){
    switch (flag){
        case 0: // 正方向
            return 1;
        case 1: // 轮子没有转
            return 0;
        case 2: // 反方向
            return -1;
    }
}


//这个和陀螺仪设置的量程有关的 转化为度每秒是/65.5 转为弧度每秒/57.3 其子65.5看MPU6050手册，STM32底层FS_SEL=1
#define GYROSCOPE_RATIO	0.00026644f // 1/65.5/57.30=0.00026644 陀螺仪原始数据换成弧度单位
//这个和陀螺仪设置的量程有关的 转化为度每秒是/65.5 转为弧度每秒/57.3 其子65.5看MPU6050手册，STM32底层FS_SEL=1
#define ACCEl_RATIO 	16384.0f  	// 量程±2g，重力加速度定义为1g等于9.8米每平方秒。

void serial_recv(SerialRxData *rx_data,size_t size){
    uint8_t recvBuffer[size];

    do{
        //头7B 使能01 X:00 00 Y:00 00 Z:00 00 X_A:FF 86 Y_A:FF D8 Z_A:41 EC X_G:00 00 Y_G:FF FE Z_G:FF FE  B:5A 28  80 尾7D

        do{  // 接收消息头
            recvBuffer[0] = 0;
            ros_ser.read(&recvBuffer[0],1);


            // 先读消息头
            if(recvBuffer[0] != 0x7B){
                //ROS_WARN("serial_rx protocol_syn_start error:%d %d",recvBuffer[0],recvBuffer[1]);
                continue;
            }
            //ROS_WARN("serial SERIAL_SYN_CODE_START  success");
            break;
        }while(1);

        // 从消息头位置，再往后读23字节
        ros_ser.read(&recvBuffer[1],24-1);

        // 判断最后一字节是否为 0x7D
        if(recvBuffer[23] != 0x7D){
            continue;
        }

        // 当走到这里，说明帧头和帧尾全部匹配正确,解析数据
        double x_vel = recvBuffer[2] << 8 | recvBuffer[3];
        double y_vel = recvBuffer[4] << 8 | recvBuffer[5];
        double z_vel = recvBuffer[6] << 8 | recvBuffer[7];

        // imu 加速度
        double x_acc = recvBuffer[8] << 8 | recvBuffer[9];
        double y_acc = recvBuffer[10] << 8 | recvBuffer[11];
        double z_acc = recvBuffer[12] << 8 | recvBuffer[13];


        x_acc/=ACCEl_RATIO;
        y_acc/=ACCEl_RATIO;
        z_acc/=ACCEl_RATIO;

        // imu 角速度
        double x_gyro = recvBuffer[14] << 8 | recvBuffer[15];
        double y_gyro = recvBuffer[16] << 8 | recvBuffer[17];
        double z_gyro = recvBuffer[18] << 8| recvBuffer[19];

        x_gyro*=GYROSCOPE_RATIO;
        y_gyro*=GYROSCOPE_RATIO;
        z_gyro*=GYROSCOPE_RATIO;
        //ROS_INFO("PROTOCOL DATA: %d %d %d %d %d %d",recvBuffer[2],recvBuffer[3],recvBuffer[4],recvBuffer[5],recvBuffer[10],recvBuffer[11]);
        ROS_INFO("recv data: %f %f %f %f %f %f %f %f %f ",x_vel,y_vel,z_vel,x_acc,y_acc,z_acc,x_gyro,y_gyro,z_gyro);

        rx_data->x_vel = x_vel;
        rx_data->y_vel = y_vel;
        rx_data->z_vel = z_vel;

        rx_data->x_acc = x_acc;
        rx_data->y_acc = y_acc;
        rx_data->z_acc = z_acc;

        rx_data->x_gyro = x_gyro;
        rx_data->y_gyro = y_gyro;
        rx_data->z_gyro = z_gyro;

        break;
    }while(1);
}

void serial_recv_pre(uint8_t *recvBuffer,size_t size){
    //uint8_t recvBuffer[size];
        //FF FE 00 01 00 01 00 01 00 01 FF 94 FF A4 41 DE FF FF FF FF FF FF
        do {
            //解析数据
            // 数据帧总长度12字节 前两个字节是帧头

            memset(recvBuffer, 0, sizeof(recvBuffer));
            // 寻找帧头 5A 5A 07 13   FC FE FD D1 02 42   00 54 02 61 00 63   E7 2B F0 AA 1F 9D   73 CF
            do {
                recvBuffer[0] = 0;
                recvBuffer[1] = 0;
                ros_ser.read(&recvBuffer[0], 1);
                if(recvBuffer[0] == 0xff){
                    ros_ser.read(&recvBuffer[1], 1);
                    if(recvBuffer[1] == 0xfe){
                        break;
                    }
                }
            } while (1);


        // 从消息头位置，再往后读23字节
        ros_ser.read(&recvBuffer[2],22-2);

        break;
    }while(1);
}



void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg){
    if(param_use_debug_cmd){
        ROS_INFO("CMD VEL LINE[%1.2F,%1.2f,%1.2f],Ang[%1.2F,%1.2f,%1.2f]",
                msg->linear.x,msg->linear.y,msg->linear.z,
                msg->angular.x,msg->angular.y,msg->angular.z);
    }
    /*uint8_t tx_data[11];
    uint8_t  transition;  //中间变量
    tx_data[0]=0X7B;//数据的第一位是帧头（固定值）
    tx_data[1] = 0x01 ; //产品型号
    tx_data[2] = 0x01 ;  //机器人使能控制标志位
    //机器人x轴的目标线速度
    transition=0;
    transition = msg->linear.x*1000; //将浮点数放大一千倍，简化传输
    tx_data[4] = transition;     //取数据的低8位
    tx_data[3] = transition>>8;  //取数据的高8位
    //机器人y轴的目标线速度
    transition=0;
    transition = msg->linear.y*1000;
    tx_data[6] = transition;
    tx_data[5] = transition>>8;
    //机器人z轴的目标角速度
    transition=0;
    transition = msg->angular.z*1000;
    tx_data[8] = transition;
    tx_data[7] = transition>>8;
    tx_data[9]=  0x00;//tx_data[1]^tx_data[2]^tx_data[3]^tx_data[4]^tx_data[5]^tx_data[6]^tx_data[7];//帧尾校验位，规则参见Check_Sum函数
    tx_data[10]=0X7D;  //数据的最后一位是帧尾（固定值）


    ros_ser.write(tx_data, sizeof(tx_data));*/

    uint8_t tx_data[10];
//    SerialTxData txdata;
    memset(tx_data,0, sizeof(tx_data));

    tx_data[0] = 0xff;
    tx_data[1] = 0xfe;

    Kinematics::RPM result = kinematics.calculateRPM(msg->linear.x,msg->angular.z);

    tx_data[2] = result.leftSignalNum;
    tx_data[3] = result.leftSignalDirect;

    tx_data[4] = result.rightSignalNum;
    tx_data[5] = result.rightSignalDirect;


    ros_ser.write(tx_data, sizeof(tx_data));

    ROS_INFO("%d %d %d %d",result.leftSignalNum,result.leftSignalDirect,result.rightSignalNum,result.rightSignalDirect);
    //ROS_INFO("cmd vel data: %d %d %d %d",txdata.a_vel,txdata.a_direct,txdata.b_vel,txdata.b_direct);
}


double limit_value(double value){
    if(value < -32760){
        return 0;
    }else if(value > 32760){
        return 0;
    }
}

int main(int argc, char** argv ) {
    ros::init(argc, argv, "heima_serial_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    nh.param<bool>("debug_imu", param_use_debug_imu, false);
    nh.param<bool>("debug_cmd", param_use_debug_cmd, false);
    nh.param<bool>("debug_vel", param_use_debug_vel, false);

    nh.param<std::string>("port", param_port_path, "/dev/ttyUSB0");
    nh.param<int>("baudrate", param_baudrate_, 115200);


    //开启一个异步的轮询器
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher pub_raw_pose = n.advertise<heima_msgs::Velocities>("raw_vel", 1000);
    ros::Publisher pub_imu = n.advertise<heima_msgs::Imu>("raw_imu", 1000);
    ros::Subscriber sub_cmd_vel = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, callback_cmd_vel);


    param_use_debug_imu = true;
    param_use_debug_cmd = true;
    param_use_debug_vel = false;


    try
    {
        ros_ser.setPort(param_port_path);
        ros_ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ros_ser.setTimeout(to);
        ros_ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port "<<param_port_path);
        return -1;
    }

    if(ros_ser.isOpen()){
        ROS_INFO_STREAM("Serial Port opened"<<param_port_path);
    }else{
        return -1;
    }

    // 使能串口通讯
    uint8_t enable_data[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    ros_ser.write(enable_data, sizeof(enable_data));



    heima_msgs::Velocities pub_msg_pose;
    heima_msgs::Imu pub_msg_imu;
    SerialRxData data;
    try {
        ros::Rate rate(45);
        while(ros::ok()){
            if(ros_ser.available()){
                //SerialRxData rx_data;
                // 读取客户端发送过来的数据
                // serial_recv(&rx_data,24);
                uint8_t recvBuffer[22];

                serial_recv_pre(recvBuffer,22);

                int l_num = (int)recvBuffer[2];
                int l_direct = (int)recvBuffer[3];

                int r_num = (int)recvBuffer[4];
                int r_direct = (int)recvBuffer[5];


                //ROS_INFO("gyroz: %d  %d  %d",data.z_high,data.z_low,gyroz);

//                int l_num = (int)data.a_vel;
//                int l_direct = (int)data.a_direct;
//
//                int r_num = (int)data.b_vel;
//                int r_direct = (int)data.b_direct;




                // 小车当前的线速度和角速度
                Kinematics::Velocity velocity = kinematics.getVelocityByNum(l_num*getDirect(l_direct),r_num*getDirect(r_direct));

                // 封装成自定义的消息
                pub_msg_pose.linear_x = velocity.linear_velocity;
                pub_msg_pose.linear_y = 0;
                pub_msg_pose.angular_z = velocity.angular_velocity;

                // 向外发布底盘的数据
                pub_raw_pose.publish(pub_msg_pose);

                short x = ((recvBuffer[10]<<8) | recvBuffer[11]);
                short y = ((recvBuffer[12]<<8) | recvBuffer[13]);
                short z = ((recvBuffer[14]<<8 | recvBuffer[15]));
                x-=32768;
                y-=32768;
                z-=32768;
////                ROS_INFO("gyroz: %d  %d  %f",recvBuffer[10],recvBuffer[10]<<8,(double)((int)recvBuffer[10]<<8 | (int)recvBuffer[11])-32768);
                pub_msg_imu.linear_acceleration.x = x;//(double)(recvBuffer[10]<<8 | recvBuffer[11]) - 32768;
                pub_msg_imu.linear_acceleration.y = y;//(double)(recvBuffer[12]<<8 | recvBuffer[13]) - 32768;
                pub_msg_imu.linear_acceleration.z = z;//(double)(recvBuffer[14]<<8 | recvBuffer[15]) - 32768;

                x = (recvBuffer[16]<<8 | recvBuffer[17]);
                y = (recvBuffer[18]<<8 | recvBuffer[19]);
                z = (recvBuffer[20]<<8 | recvBuffer[21]);

                x-=32768;
                y-=32768;
                z-=32768;

                pub_msg_imu.angular_velocity.x = x;
                pub_msg_imu.angular_velocity.y = y;
                pub_msg_imu.angular_velocity.z = z;

                pub_msg_imu.magnetic_field.x = 0;
                pub_msg_imu.magnetic_field.y = 0;
                pub_msg_imu.magnetic_field.z = 0;

                pub_imu.publish(pub_msg_imu);
                ROS_INFO("PUB IMU DATA");

            }
//        ros::spinOnce();
            rate.sleep();
        }
    }catch (serial::IOException& E){
        ros_ser.close();
        ROS_ERROR("SERIAL ERROR");
    }



}

