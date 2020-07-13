#include <heima_msgs/Velocities.h>
#include <heima_msgs/Imu.h>
#include "wheeltec_robot.h"

sensor_msgs::Imu Mpu6050;//实例化IMU对象
/**************************************
Date: May 31, 2020
Function: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/
int main(int argc, char **argv) {
    ros::init(argc, argv, "wheeltec_robot");//ROS初始化 并设置节点名称，可修改
    ROS_INFO("wheeltec_robot node has turned on ");//显示状态
    turn_on_robot Robot_Control; //实例化一个对象
    Robot_Control.Control();  //循环执行数据采集和发布topic等操作
    return 0;
}

/**************************************
Date: June 29, 2020
Function: 数据传输转换函数
***************************************/
short turn_on_robot::IMU_Trans(uint8_t Data_High, uint8_t Data_Low) {
    short transition_16;
    transition_16 = 0;
    transition_16 |= Data_High << 8;
    transition_16 |= Data_Low;
    return transition_16;
}

float turn_on_robot::Odom_Trans(uint8_t Data_High, uint8_t Data_Low) {
    float data_return;
    short transition_16;
    transition_16 = 0;
    transition_16 |= Data_High << 8;  //获取数据的高8位
    transition_16 |= Data_Low;     //获取数据的低8位
    data_return = (transition_16 / 1000) + (transition_16 % 1000) * 0.001; //(发送端将数据放大1000倍发送，这里需要将数据单位还原)
    return data_return;
}

/**************************************
Date: June 29, 2020
Function: 订阅回调函数Callback，根据订阅的指令向串口发指令控制下位机
***************************************/
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::Twist &twist_aux) {

    isReading = true;


    short transition;  //中间变量
    Send_Data.tx[0] = FRAME_HEADER;//帧头 固定值
    Send_Data.tx[1] = 1; //产品型号
    Send_Data.tx[2] = 0;  //机器人使能控制标志位
    //机器人x轴的目标线速度
    transition = 0;
    transition = twist_aux.linear.x * 1000; //将浮点数放大一千倍，简化传输
    Send_Data.tx[4] = transition;     //取数据的低8位
    Send_Data.tx[3] = transition >> 8;  //取数据的高8位
    //机器人y轴的目标线速度
    transition = 0;
    transition = twist_aux.linear.y * 1000;
    Send_Data.tx[6] = transition;
    Send_Data.tx[5] = transition >> 8;
    //机器人z轴的目标角速度
    transition = 0;
    transition = twist_aux.angular.z * 1000;
    Send_Data.tx[8] = transition;
    Send_Data.tx[7] = transition >> 8;

    Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK);//帧尾校验位，规则参见Check_Sum函数
    Send_Data.tx[10] = FRAME_TAIL;  //数据的最后一位是帧尾（固定值）
    try {
        // if(Receive_Data.Flag_Stop==0)
        Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx)); //向串口发数据
        ROS_INFO_STREAM("New control command:"<<twist_aux.linear.x); //显示受到了新的控制指令
    }
    catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to send data through serial port"); //如果try失败,打印错误信息
    }
    isReading = false;
   // cv.notify_all();
}

/**************************************
Date: May 31, 2020
Function: Amcl相关回调函数Callback
***************************************/
void turn_on_robot::Cmd_Amclvel_Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &Pose) {
    geometry_msgs::Pose amclpose;//订阅机器人的姿态信息
    amclpose.position.x = Pose->pose.pose.position.x;
    amclpose.position.y = Pose->pose.pose.position.y;
    amclpose.orientation = Pose->pose.pose.orientation;

    ROS_INFO("X:%f Y:%f orientation x:%f y:%f z:%f w:%f", amclpose.position.x, amclpose.position.y,
             amclpose.orientation.x, amclpose.orientation.y, amclpose.orientation.z, amclpose.orientation.w);
    //float temp = tf::getYaw(Amclpose.orientation);
}

/**************************************
Date: May 31, 2020
Function: 发布IMU数据
***************************************/
void turn_on_robot::Publish_ImuSensor() {


    heima_msgs::Imu imu_data_pub;

    imu_data_pub.angular_velocity.x = Mpu6050.angular_velocity.x;
    imu_data_pub.angular_velocity.y = Mpu6050.angular_velocity.y;
    imu_data_pub.angular_velocity.z = Mpu6050.angular_velocity.z;

    imu_data_pub.linear_acceleration.x = Mpu6050.linear_acceleration.x;
    imu_data_pub.linear_acceleration.y = Mpu6050.linear_acceleration.y;
    imu_data_pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;
    imu_publisher.publish(imu_data_pub);
}

/**************************************
Date: May 31, 2020
Function: 发布里程计相关信息
***************************************/
void turn_on_robot::Publish_Vel() {

    // 发布下位机过来的速度信息
    heima_msgs::Velocities vel_data_pub;


    vel_data_pub.linear_x = Robot_Vel.X;
    vel_data_pub.linear_y = Robot_Vel.Y;
    vel_data_pub.angular_z = Robot_Vel.Z;
    vel_publisher.publish(vel_data_pub);
}

/**************************************
Date: May 31, 2020
Function: 发布电压相关信息
***************************************/
void turn_on_robot::Publish_Voltage() {
    std_msgs::Float32 voltage_msgs;//定义电源电压发布topic的数据类型std_msgs::Float32
    static float Count_Voltage_Pub = 0;
    if (Count_Voltage_Pub++ > 10) {
        Count_Voltage_Pub = 0;
        voltage_msgs.data = Power_voltage;//电源供电的电压获取
        voltage_publisher.publish(voltage_msgs);//发布电源电压话题单位V
    }
}

/**************************************
Date: June 29, 2020
Function: 串口通讯校验函数，数据包除最后一个字节，其他的全部数据按位异或的结果作为帧尾
***************************************/
unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number, unsigned char mode) {
    unsigned char check_sum = 0, k;

    if (mode == 0) //接收数据
    {
        for (k = 0; k < Count_Number; k++)//Count_Number是接收数组位数减1
        {
            check_sum = check_sum ^ Receive_Data.rx[k];//按位异或
        }
    }
    if (mode == 1) //发送数据
    {
        for (k = 0; k < Count_Number; k++)//Count_Number是发送数组位数减1
        {
            check_sum = check_sum ^ Send_Data.tx[k];//按位异或
        }
    }
    return check_sum;//返回结果
}

/**************************************
Date: June 29, 2020
Function: 从串口读取数据 IMU是short类型的原始数据，单位需要结合MPU6050手册转化
***************************************/
bool turn_on_robot::Get_Sensor_Data() {
    short transition_16;  //中间变量
    std::size_t buffer_size = 48;
    uint8_t buffer[buffer_size];

    int length = Stm32_Serial.read(buffer, buffer_size);
    uint8_t start_index = 0;
    uint8_t i = 0;
    if(length ==0){
        ROS_ERROR("length : %d", length);
    }
    // 找到帧头
    while (i < length) {
        if (buffer[i] == FRAME_HEADER) {
            Receive_Data.Frame_Header = buffer[i];
            start_index = i;
            break;
        }
        i++;
    }

    // 判断帧尾
    if (buffer[start_index + 23] != FRAME_TAIL) {
        return false;
    }
    // 将buffer中的数据复制到recevie中
    memcpy(&Receive_Data.rx,&buffer, sizeof(buffer));

    // 如果走到这里，说明这帧数据没有问题
    Receive_Data.Frame_Tail = buffer[ 23];

    if (Receive_Data.rx[ 22] == Check_Sum(22, READ_DATA_CHECK))//校验位检测
    {
        Receive_Data.Flag_Stop = Receive_Data.rx[ 1];//停止位
        Robot_Vel.X = Odom_Trans(Receive_Data.rx[ 2], Receive_Data.rx[ 3]); //获取底盘X方向速度
        Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4],
                                 Receive_Data.rx[5]); //获取底盘Y方向速度//Y速度仅在全向移动机器人底盘有效
        Robot_Vel.Z = -Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7]); //获取底盘Z方向速度
        //Robot_Vel.Z = Mpu6050.angular_velocity.z
        Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8],
                                               Receive_Data.rx[9]);//获取IMU的X轴加速度
        Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10],
                                               Receive_Data.rx[11]);//获取IMU的X轴加速度
        Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[ 12],
                                               Receive_Data.rx[13]);//获取IMU的X轴加速度
        Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14],
                                              Receive_Data.rx[15]);//获取IMU的X轴角速度
        Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16],
                                              Receive_Data.rx[17]);//获取IMU的X轴角速度
        Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18],
                                              Receive_Data.rx[19]);//获取IMU的X轴角速度
        //线性加速度单位转化，和STM32 MPU6050初始化的时候的量程有关
        Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;
        //陀螺仪单位转化，和STM32底层有关，这里MPU6050的陀螺仪的量程是正负500
        //因为机器人一般Z轴速度不快，降低量程可以提高精度
        Mpu6050.angular_velocity.x = Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.y = Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.z = Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;
        //获取电池电压
        transition_16 = 0;
        transition_16 |= Receive_Data.rx[20] << 8;
        transition_16 |= Receive_Data.rx[21];
        Power_voltage = transition_16 / 1000 + (transition_16 % 1000) * 0.001;//(发送端将数据放大1000倍发送，这里需要将数据单位还原)
       // ROS_INFO("imu sensor publish");
	return true;
    }

    return false;


    /*do{
        do{
            // 查找帧头
            Receive_Data.rx[0] = 0;
            // 先读取 1 位
            Stm32_Serial.read(&Receive_Data.rx[0],1);

            // 判断是否为帧头
            if (Receive_Data.rx[0] == FRAME_HEADER ){
                Receive_Data.Frame_Header= Receive_Data.rx[0];
                break;
            }
            continue;
        }while(1);

        // 读取剩余的数据
        Stm32_Serial.read(&Receive_Data.rx[1],RECEIVE_DATA_SIZE-1);
        // 判断帧尾
        if(Receive_Data.rx[23] != FRAME_TAIL){
              continue;
        }

        // 如果走到这里，说明这帧数据没有问题
        Receive_Data.Frame_Tail= Receive_Data.rx[23];  //数据的最后一位是帧尾（数据校验位）

        if (Receive_Data.rx[22] == Check_Sum(22,READ_DATA_CHECK))//校验位检测
        {
            Receive_Data.Flag_Stop=Receive_Data.rx[1];//停止位
            Robot_Vel.X = Odom_Trans(Receive_Data.rx[2],Receive_Data.rx[3]); //获取底盘X方向速度
            Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4],Receive_Data.rx[5]); //获取底盘Y方向速度//Y速度仅在全向移动机器人底盘有效
            Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6],Receive_Data.rx[7]); //获取底盘Z方向速度
            //Robot_Vel.Z = Mpu6050.angular_velocity.z
            Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8],Receive_Data.rx[9]);//获取IMU的X轴加速度
            Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10],Receive_Data.rx[11]);//获取IMU的X轴加速度
            Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12],Receive_Data.rx[13]);//获取IMU的X轴加速度
            Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14],Receive_Data.rx[15]);//获取IMU的X轴角速度
            Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16],Receive_Data.rx[17]);//获取IMU的X轴角速度
            Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18],Receive_Data.rx[19]);//获取IMU的X轴角速度
            //线性加速度单位转化，和STM32 MPU6050初始化的时候的量程有关
            Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
            Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
            Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;
            //陀螺仪单位转化，和STM32底层有关，这里MPU6050的陀螺仪的量程是正负500
            //因为机器人一般Z轴速度不快，降低量程可以提高精度
            Mpu6050.angular_velocity.x =  Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
            Mpu6050.angular_velocity.y =  Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
            Mpu6050.angular_velocity.z =  Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;
            //获取电池电压
            transition_16 = 0;
            transition_16 |=  Receive_Data.rx[20]<<8;
            transition_16 |=  Receive_Data.rx[21];
            Power_voltage = transition_16/1000+(transition_16 % 1000)*0.001;//(发送端将数据放大1000倍发送，这里需要将数据单位还原)
            return true;
        }
        return false;
    }while(1);*/

    /* Stm32_Serial.read(Receive_Data.rx,sizeof (Receive_Data.rx));//读串口数据
     Receive_Data.Frame_Header= Receive_Data.rx[0]; //数据的第一位是帧头（固定值）
     Receive_Data.Frame_Tail= Receive_Data.rx[23];  //数据的最后一位是帧尾（数据校验位）
    if (Receive_Data.Frame_Header == FRAME_HEADER )//判断帧头
     {
       if (Receive_Data.Frame_Tail == FRAME_TAIL) //判断帧尾
       {
         if (Receive_Data.rx[22] == Check_Sum(22,READ_DATA_CHECK))//校验位检测
         {
           Receive_Data.Flag_Stop=Receive_Data.rx[1];//停止位
           Robot_Vel.X = Odom_Trans(Receive_Data.rx[2],Receive_Data.rx[3]); //获取底盘X方向速度
           Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4],Receive_Data.rx[5]); //获取底盘Y方向速度//Y速度仅在全向移动机器人底盘有效
           Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6],Receive_Data.rx[7]); //获取底盘Z方向速度
           //Robot_Vel.Z = Mpu6050.angular_velocity.z
           Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8],Receive_Data.rx[9]);//获取IMU的X轴加速度
           Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10],Receive_Data.rx[11]);//获取IMU的X轴加速度
           Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12],Receive_Data.rx[13]);//获取IMU的X轴加速度
           Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14],Receive_Data.rx[15]);//获取IMU的X轴角速度
           Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16],Receive_Data.rx[17]);//获取IMU的X轴角速度
           Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18],Receive_Data.rx[19]);//获取IMU的X轴角速度
           //线性加速度单位转化，和STM32 MPU6050初始化的时候的量程有关
           Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
           Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
           Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;
           //陀螺仪单位转化，和STM32底层有关，这里MPU6050的陀螺仪的量程是正负500
           //因为机器人一般Z轴速度不快，降低量程可以提高精度
           Mpu6050.angular_velocity.x =  Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
           Mpu6050.angular_velocity.y =  Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
           Mpu6050.angular_velocity.z =  Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;
           //获取电池电压
           transition_16 = 0;
           transition_16 |=  Receive_Data.rx[20]<<8;
           transition_16 |=  Receive_Data.rx[21];
           Power_voltage = transition_16/1000+(transition_16 % 1000)*0.001;//(发送端将数据放大1000倍发送，这里需要将数据单位还原)
           return true;
        }
       }
     }
    return false;*/
}

/**************************************
Date: May 31, 2020
Function: 这是相关控制代码，代码循环执行
***************************************/
void turn_on_robot::Control() {
	isReading = false;
    _Last_Time = ros::Time::now();
    ros::Rate rate(30);
    while (ros::ok()) {
//    _Now = ros::Time::now();
//    Sampling_Time = (_Now - _Last_Time).toSec();
        try {
//		unique_lock<mutex> lock(m);
	        if(isReading){
        	   continue;
			//cv.wait(lock);
    		}

                //isReading = true;
                Get_Sensor_Data();

                Publish_Vel();        //发布里程计话题
                Publish_ImuSensor();  //发布话题
                Publish_Voltage();
               
               // isReading = false;
	      
               // cv.notify_all();
		//ROS_ERROR("SENSOR publish");
        } catch (exception e) {
            ROS_INFO_STREAM("heima serial:" << e.what());
        }

       rate.sleep();
       // ros::spinOnce();
        //Sampling_time是采样时间，虽然下位机发送的数据频率是固定的，这里计算里程增量以ROS系统的时间更加可靠精确。
        /*if (true == Get_Sensor_Data())  //从串口读取下位机法过来的全部数据
        {
          *//*Robot_Pos.X+=(Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;//计算x方向的位移
      Robot_Pos.Y+=(Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;//计算y方向的位移， 
      Robot_Pos.Z+=Robot_Vel.Z * Sampling_Time; //角位移  
      Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,\
                Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);//四元数解算*//*
      Publish_Odom();        //发布里程计话题
      Publish_ImuSensor();  //发布话题    
      Publish_Voltage(); //发布电源电压
    }
    _Last_Time = _Now;//记录时间*/
        //ros::spinOnce();//循环等待回调函数
//
    }


}

/**************************************
Date: May 31, 2020
Function: 构造函数, 只执行一次，用于初始化
***************************************/
turn_on_robot::turn_on_robot() : Sampling_Time(0), Power_voltage(0) {
    memset(&Robot_Pos, 0, sizeof(Robot_Pos));
    memset(&Robot_Vel, 0, sizeof(Robot_Vel));
    memset(&Receive_Data, 0, sizeof(Receive_Data)); //构造函数初始化
    memset(&Send_Data, 0, sizeof(Send_Data));
    memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));
    ros::NodeHandle private_nh("~");
    //把以上的类成员参数注册到参数服务器，这样在launch文件里面即可修改
    //3个入口参数分别对应：参数服务器上的名称  参数变量名  初始值
    private_nh.param<std::string>("usart_port_name", usart_port_name, "/dev/wheeltec_controller"); //固定串口
    private_nh.param<int>("serial_baud_rate", serial_baud_rate, 115200); //和下位机底层波特率115200 不建议更高的波特率了
    private_nh.param<std::string>("smoother_cmd_vel", smoother_cmd_vel, "/smoother_cmd_vel");//平滑控制指令
    private_nh.param<std::string>("robot_frame_id", robot_frame_id, "base_link");//ID


    //开启一个异步的轮询器
   spinner = new ros::AsyncSpinner(1);
    spinner->start();



    //发布3个话题，订阅2个话题
    voltage_publisher = n.advertise<std_msgs::Float32>("/PowerVoltage", 10);//电池电压数据发布
    vel_publisher = n.advertise<heima_msgs::Velocities>("raw_vel", 100);
    imu_publisher = n.advertise<heima_msgs::Imu>("raw_imu", 100);

    //odom_publisher = n.advertise<nav_msgs::Odometry>("odom", 50);//里程计数据发布
    //imu_publisher  = n.advertise<sensor_msgs::Imu>("/mobile_base/sensors/imu_data", 20);//IMU数据发布
    Cmd_Vel_Sub = n.subscribe("cmd_vel", 5, &turn_on_robot::Cmd_Vel_Callback, this);//因为官方的平滑包只支持X和W，没有Y，所以这里不使用平滑包
    // Cmd_Vel_Sub = n.subscribe(smoother_cmd_vel, 100, &turn_on_robot::Cmd_Vel_Callback, this);//订阅smoother_cmd_vel话题并控制机器人//差速
    Amcl_Sub = n.subscribe("amcl_pose", 100, &turn_on_robot::Cmd_Amclvel_Callback, this);//自适应蒙特卡洛定位需要的数据
    ROS_INFO_STREAM("Data ready");//ready显示状态
    //初始化串口
    try {
        Stm32_Serial.setPort(usart_port_name);//选择哪个口，如果选择的口没有接串口外设初始化会失败
        Stm32_Serial.setBaudrate(serial_baud_rate);//设置波特率
        serial::Timeout _time = serial::Timeout::simpleTimeout(2000);//超时等待
        Stm32_Serial.setTimeout(_time);
        Stm32_Serial.open();//串口开启
    }
    catch (serial::IOException &e) {
        ROS_ERROR_STREAM(
                "wheeltec_robot can not open serial port,Please check the serial port cable! ");//如果try失败，打印错误信息
    }
    if (Stm32_Serial.isOpen()) {
        ROS_INFO_STREAM("wheeltec_robot serial port opened");//开启成功
    } else {
    }

    /*new std::thread(&turn_on_robot::do_recv, this);
    new std::thread(&turn_on_robot::do_parse, this);*/
}

/**************************************
Date: May 31, 2020
Function: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
turn_on_robot::~turn_on_robot() {
    Stm32_Serial.close();//关闭串口
    ROS_INFO_STREAM("Shutting down");//close
    spinner->stop();
    delete spinner;
}

void turn_on_robot::do_recv() {
    uint8_t buffer[128];
    ros::Rate ros_rate(30);
    while (ros::ok()) {
        isReading = true;
        size_t len = Stm32_Serial.read(buffer, 128);

        TMP_DATA tmp;
        tmp.data = new uint8_t[len];
        tmp.len = len;
        memcpy(&tmp.data[0], &buffer[0], len);

        queue.put(tmp);
        isReading = false;
        cv.notify_all();

        ros_rate.sleep();
    }
}

void turn_on_robot::do_parse() {

    ros::Rate ros_rate(30);
    while (ros::ok()) {
        TMP_DATA tmp = queue.take();

        uint8_t *data = tmp.data;
        size_t len = tmp.len;

        parse_serial(data,len,ros_rate);



        delete tmp.data;

    }
}

bool turn_on_robot::parse_serial(uint8_t* buffer, std::size_t length,ros::Rate &ros_rate)  {
    short transition_16;  //中间变量
    uint8_t start_index = 0;
    uint8_t i = 0;




    //ROS_INFO("length : %d", length);
    // 可能有多帧数据
    while(i < length - 23){
        // 找帧头

        if (buffer[i] == FRAME_HEADER) {
            Receive_Data.Frame_Header = buffer[i];
            start_index = i;
            i++;
        }else{
            i++;
            continue;
        }



        // 判断帧尾
        if (buffer[start_index + 23] != FRAME_TAIL) {
            continue;
        }

        // 如果走到这里，说明这帧数据没有问题
        Receive_Data.Frame_Tail = buffer[start_index + 23];

        if (Receive_Data.rx[start_index + 22] == Check_Sum(22, READ_DATA_CHECK))//校验位检测
        {
            Receive_Data.Flag_Stop = Receive_Data.rx[start_index + 1];//停止位
            Robot_Vel.X = Odom_Trans(Receive_Data.rx[start_index + 2], Receive_Data.rx[start_index + 3]); //获取底盘X方向速度
            Robot_Vel.Y = Odom_Trans(Receive_Data.rx[start_index + 4],
                                     Receive_Data.rx[start_index + 5]); //获取底盘Y方向速度//Y速度仅在全向移动机器人底盘有效
            Robot_Vel.Z = Odom_Trans(Receive_Data.rx[start_index + 6], Receive_Data.rx[start_index + 7]); //获取底盘Z方向速度
            //Robot_Vel.Z = Mpu6050.angular_velocity.z
            Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[start_index + 8],
                                                   Receive_Data.rx[start_index + 9]);//获取IMU的X轴加速度
            Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[start_index + 10],
                                                   Receive_Data.rx[start_index + 11]);//获取IMU的X轴加速度
            Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[start_index + 12],
                                                   Receive_Data.rx[start_index + 13]);//获取IMU的X轴加速度
            Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[start_index + 14],
                                                  Receive_Data.rx[start_index + 15]);//获取IMU的X轴角速度
            Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[start_index + 16],
                                                  Receive_Data.rx[start_index + 17]);//获取IMU的X轴角速度
            Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[start_index + 18],
                                                  Receive_Data.rx[start_index + 19]);//获取IMU的X轴角速度
            //线性加速度单位转化，和STM32 MPU6050初始化的时候的量程有关
            Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
            Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
            Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;
            //陀螺仪单位转化，和STM32底层有关，这里MPU6050的陀螺仪的量程是正负500
            //因为机器人一般Z轴速度不快，降低量程可以提高精度
            Mpu6050.angular_velocity.x = Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
            Mpu6050.angular_velocity.y = Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
            Mpu6050.angular_velocity.z = Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;
            //获取电池电压
            transition_16 = 0;
            transition_16 |= Receive_Data.rx[start_index + 20] << 8;
            transition_16 |= Receive_Data.rx[start_index + 21];
            Power_voltage = transition_16 / 1000 + (transition_16 % 1000) * 0.001;//(发送端将数据放大1000倍发送，这里需要将数据单位还原)


            Publish_ImuSensor();
            Publish_Vel();
            ros_rate.sleep();
            ROS_INFO("SENSOR publish");
        }
        i+=23;
        continue;
    }





}

