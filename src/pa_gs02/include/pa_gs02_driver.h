#ifndef PA_GS02_DRIVER_H
#define PA_GS02_DRIVER_H

#include <ros/ros.h>
#include <sys/time.h>
#include<chrono>
#include <sensor_msgs/Imu.h>
// #include <boost/asio.hpp>        
// #include <boost/bind.hpp>
#include <tf/tf.h>
#include <serial/serial.h>  

#include <std_msgs/Empty.h>

#include <std_msgs/String.h>
#include <eigen3/Eigen/Dense>
#include <crc16.h>

#define G 9.80665

using namespace std;

int last_length=0;
int mycount =0;
class Parse_GPS
{
private:


    ros::NodeHandle nh;
    ros::NodeHandle nh_get;
    ros::Publisher read_pub;
    serial::Serial ser_IMU; //声明串口对象
    string imuport;  // imu端口名
    std::chrono::steady_clock::time_point now;
    std::chrono::steady_clock::time_point last;

    //   #include <imu_driver/IMU_DATA.h>

    const float MinLength = 76;

    struct IMU_DATA_structure
    {
    float Gyro_x; // Gyro x axis
    float Gyro_y; // Gyro y axis
    float Gyro_z; // Gyro z axis

    float Acc_x; // Accel x axis
    float Acc_y; // Accel y axis
    float Acc_z; // Accel z axis

    float Magn_x;
    float Magn_y;
    float Magn_z;

    float Hbar; //大气压 1hbar=100pa

    float Pitch; // Pitch
    float Roll; // Roll
    float Yaw; // Yaw

    float Vel_E; // 地面速度 E
    float Vel_N; // 地面速度 N
    float Vel_U; // 地面速度 U

    double Longitude;
    double Latitude;
    float  Altitude;

    unsigned char flag_frequency;
    unsigned char flag_Magn;
    unsigned char flag_Hbar;
    unsigned char Satellites_num;

    unsigned char Pos_type;

    double IMU_Time;  // 单位ms
    float Temp; // Temperature
    };

    bool initialize_systeam=false;

    sensor_msgs::Imu result; 
    struct IMU_DATA_structure imuData;

public:

    unsigned char *last_buffer;
    
    
    Parse_GPS():nh_get("~")
    {
        // memset(last_buffer,0,78);
        read_pub = nh.advertise<sensor_msgs::Imu>("/imu/data",1);//串口接收  //发布数据
        
        nh_get.getParam("imuport",imuport);
        try
        {
        //设置串口属性，并打开串口
            ser_IMU.setPort(imuport);//串口名称
            ser_IMU.setBaudrate(460800);//波特率
            // ser_IMU.setStopbits(stopbits_one);
            ser_IMU.setFlowcontrol(serial::flowcontrol_none);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser_IMU.setTimeout(to);

            ser_IMU.open();
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR_STREAM("Unable to open port ");
            exit(0);
        }
        //检测串口是否已经打开，并给出提示信息
        if(ser_IMU.isOpen())
        {
            ROS_INFO_STREAM("Serial Port initialized");
        }
        else
        {
            exit(0);
        }


        ros::Rate loop_rate(400);//指定循环的频率 
        // std_msgs::String imu_str2;
        // imu_str2.data = ser_IMU.read(ser_IMU.available());
        int count_num=0;
        while(ros::ok())
        {
            now = std::chrono::steady_clock::now();
            
            double t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(now - last).count();
            last = now;
            // ROS_INFO_STREAM("*********************: ");
            if(ser_IMU.available()){
                // ROS_INFO_STREAM("Reading from serial port\n");
                   std_msgs::String imu_str;
                // if(ser_IMU.available()!=78){
                    // imu_str.data = ser_IMU.readline();
                // }
                // else{
                if(!initialize_systeam){
                    unsigned char *temp;
                    do{
                        imu_str.data = ser_IMU.read(1);
                        temp=(unsigned char *)imu_str.data.c_str();
                    }
                    while(*temp!=0xDA);
                    imu_str.data = ser_IMU.read(1);
                    temp=(unsigned char *)imu_str.data.c_str();
                    if(*temp==0x61){
                        initialize_systeam=true;
                        ser_IMU.read(76); 
                        cout<<"systeam initializes successful"<<endl;
                    }
                    
                 
                }
                else
                {
                
                    imu_str.data = ser_IMU.read(78); 
                    ParseImuData(&imuData,(unsigned char *)imu_str.data.c_str(),imu_str.data.length());
               
                     
                }
                
               
                // }
                   
                    
                // ROS_INFO_STREAM("Read: " << imu_str.data.length());
                // cout<< imu_str.data.c_str()<<endl;+
               
            }

            //处理ROS的信息，比如订阅消息,并调用回调函数
            ros::spinOnce();
            loop_rate.sleep();

        }
    }
    void covert_ImuData_topic(IMU_DATA_structure imuData,sensor_msgs::Imu &topic_data){

    topic_data.header.stamp = ros::Time::now();
    topic_data.header.frame_id = "imu_link";

    topic_data.linear_acceleration.x=  imuData.Acc_x * G;
    topic_data.linear_acceleration.y=  imuData.Acc_y *G;
    topic_data.linear_acceleration.z=  imuData.Acc_z *G;

    // if(fabs(topic_data.linear_acceleration.x)<0.1) topic_data.linear_acceleration.x=0;
    // if(fabs(topic_data.linear_acceleration.y)<0.1) topic_data.linear_acceleration.y=0;
    // if(fabs(topic_data.linear_acceleration.z)<0.1) topic_data.linear_acceleration.z=0;
    // Gyro x, y, z
    topic_data.angular_velocity.x=  imuData.Gyro_x*M_PI/180.0;
    topic_data.angular_velocity.y=  imuData.Gyro_y*M_PI/180.0;
    topic_data.angular_velocity.z=  imuData.Gyro_z*M_PI/180.0;

    topic_data.orientation_covariance={0.0012, 0.0,    0.0,
                                    0.0,    0.0012, 0.0,
                                    0.0,    0.0,    0.0012};
    topic_data.angular_velocity_covariance={0.0004, 0.0,    0.0,
                                        0.0,    0.0004, 0.0,
                                        0.0,    0.0,    0.0004};
    topic_data.linear_acceleration_covariance={0.0096, 0.0,    0.0,
                                            0.0,    0.0096, 0.0,
                                            0.0,    0.0,    0.0096};


    // Roll, Pitch, Yaw
    // topic_data.Roll= imuData->Roll ;
    // topic_data.Pitch=  imuData->Pitch ;
    // topic_data.Yaw=  imuData->Yaw;
    // topic_data.Temp=  imuData->Temp;
    float radian_x=imuData.Roll*M_PI/180.0;
    float radian_y=imuData.Pitch*M_PI/180.0;
    float radian_z=imuData.Yaw*M_PI/180.0;
    topic_data.orientation = tf::createQuaternionMsgFromRollPitchYaw(radian_x,radian_y,radian_z);

    }



    // 参数说明
    // imuData: 用于存储IMU数据的变量
    // buffer: IMU具体数据缓冲区
    // length: 这次解析数据的长度
    bool ParsePacketB4(IMU_DATA_structure* imuData, unsigned char* buffer, int length)
    {
      

        if (imuData == NULL || buffer == NULL || length != MinLength)
        {
            cout<<"输入错误"<<endl;
            return false;
        }

        // Gyro x, y, z
        int gx = ((0xff &(char)buffer[0]) << 16) | ((0xff &(char)buffer[1]) << 8) | (0xff &(char)buffer[2]);
        int gy = ((0xff &(char)buffer[3]) << 16) | ((0xff &(char)buffer[4]) << 8) | (0xff &(char)buffer[5]);
        int gz = ((0xff &(char)buffer[6]) << 16) | ((0xff &(char)buffer[7]) << 8) | (0xff &(char)buffer[8]);
        imuData->Gyro_x=(float)((gx<<8)>>8)*1e-3f;
        imuData->Gyro_y=(float)((gy<<8)>>8)*1e-3f;
        imuData->Gyro_z=(float)((gz<<8)>>8)*1e-3f;
        // cout<<"The Gyro of xyz is:"<<imuData->Gyro_x<<","<<imuData->Gyro_y<<","<<imuData->Gyro_z<<endl;

        // Accel x, y, z
        int Ax = ((0xff &(char)buffer[9]) << 16) | ((0xff &(char)buffer[10]) << 8) | (0xff &(char)buffer[11]);
        int Ay = ((0xff &(char)buffer[12]) << 16) | ((0xff &(char)buffer[13]) << 8) | (0xff &(char)buffer[14]);
        int Az = ((0xff &(char)buffer[15]) << 16) | ((0xff &(char)buffer[16]) << 8) | (0xff &(char)buffer[17]);
        imuData->Acc_x =(float)((Ax<<8)>>8)*1e-5f;
        imuData->Acc_y =(float)((Ay<<8)>>8)*1e-5f;
        imuData->Acc_z =(float)((Az<<8)>>8)*1e-5f;
        // cout<<"The ACC of xyz is:"<<imuData->Acc_x<<","<<imuData->Acc_y<<","<<imuData->Acc_z<<endl;

        // Magn x, y, z
        short int Mx = ((0xff &(char)buffer[18]) << 8) | (0xff &(char)buffer[19]);
        short int My = ((0xff &(char)buffer[20]) << 8) | (0xff &(char)buffer[21]);
        short int Mz = ((0xff &(char)buffer[22]) << 8) | (0xff &(char)buffer[23]);
        imuData->Magn_x =(float)(Mx)*1e-2f;
        imuData->Magn_y =(float)(My)*1e-2f;
        imuData->Magn_z =(float)(Mz)*1e-2f;

        // Hbar
        int hbar = ((0xff &(char)buffer[24]) << 16) | ((0xff &(char)buffer[25]) << 8) | (0xff &(char)buffer[26]);
        imuData->Hbar = (float)((hbar<<8)>>8)*1e-2f;
        // cout<<"当前的大气压是："<<imuData->Hbar/1000.<<endl;

        //  Pitch, Roll,Yaw
        short int pitch = ((0xff &(char)buffer[27]) << 8) | (0xff &(char)buffer[28]);
        short int roll = ((0xff &(char)buffer[29]) << 8) | (0xff &(char)buffer[30]);
        short int yaw = ((0xff &(char)buffer[31]) << 8) | (0xff &(char)buffer[32]);
        imuData->Roll = (float)(pitch)*1e-2f;   // pitch和roll是互换
        imuData->Pitch = (float)(roll)*1e-2f;
        imuData->Yaw =(float)(yaw)*1e-2f;
        //imuData->Yaw +=110.0;
        while(imuData->Yaw > 180.0f){
            imuData->Yaw = imuData->Yaw - 360.f;
        }
        while(imuData->Yaw<-180.0f){
            imuData->Yaw = imuData->Yaw + 360.f;
        }
        cout<<"The RPY is:"<<imuData->Roll<<","<<imuData->Pitch<<","<<imuData->Yaw<<endl;

        // ground speed
        int vel_e = ((0xff &(char)buffer[33]) << 16) | ((0xff &(char)buffer[34]) << 8) | (0xff &(char)buffer[35]);
        int vel_n = ((0xff &(char)buffer[36]) << 16) | ((0xff &(char)buffer[37]) << 8) | (0xff &(char)buffer[38]);
        int vel_u = ((0xff &(char)buffer[39]) << 16) | ((0xff &(char)buffer[40]) << 8) | (0xff &(char)buffer[41]);
        imuData->Vel_E =(float)((vel_e<<8)>>8)*1e-4f;
        imuData->Vel_N =(float)((vel_n<<8)>>8)*1e-4f;
        imuData->Vel_U =(float)((vel_u<<8)>>8)*1e-4f;
        // cout<<"The ground speed of ENU is:"<<imuData->Vel_E<<","<<imuData->Vel_N<<","<<imuData->Vel_U<<endl;

        // Global Position
        int longitude = ((0xff &(char)buffer[42]) << 24) |  ((0xff &(char)buffer[43]) << 16) | ((0xff &(char)buffer[44]) << 8) | (0xff &(char)buffer[45]);
        int latitude = ((0xff &(char)buffer[46]) << 24) |  ((0xff &(char)buffer[47]) << 16) | ((0xff &(char)buffer[48]) << 8) | (0xff &(char)buffer[49]);
        int altitude = ((0xff &(char)buffer[50]) << 16) | ((0xff &(char)buffer[51]) << 8) | (0xff &(char)buffer[52]);
        imuData->Longitude = (double)(longitude)*1e-7;
        imuData->Latitude = (double)(latitude)*1e-7;
        imuData->Altitude = (float)((altitude<<8)>>8)*1e-2;
        // cout<<"The Global Position of Lon/Lat/Alt is:"<<imuData->Longitude<<","<<imuData->Latitude<<","<<imuData->Altitude<<endl;

        // flag  
        // TODO 这里是空的，看看是不是因为没天线的原因
        imuData->flag_frequency = 0xc0&(unsigned char)buffer[53];
        imuData->flag_Magn = 0x20&(unsigned char)buffer[53];
        imuData->flag_Hbar = 0x10&(unsigned char)buffer[53];
        imuData->Satellites_num = 0x0f&(unsigned char)buffer[53];
        // cout<<buffer[53]<<endl;
        // cout<<imuData->flag_frequency<<","<<imuData->flag_Magn<<","<<imuData->flag_Hbar<<","<<imuData->Satellites_num<<endl;

        // TODO 这里是空的，看看是不是因为没天线的原因
        imuData->Pos_type = (unsigned char)buffer[54];
        // cout<<buffer[54]<<endl;

        // 这个记录的是系统启动开始的计时 没什么用
        int imu_time1 = (0xff &(char)buffer[60])>>7;
        int imu_time2 = ((0xff &(char)buffer[61]) << 24) |  ((0xff &(char)buffer[62]) << 16) | ((0xff &(char)buffer[63]) << 8) | (0xff &(char)buffer[64]);
        imuData->IMU_Time = (double)((imu_time1<<25) | (imu_time2))*1e-1;
        // cout<<imu_time1<<","<<imu_time2<<","<<imuData->IMU_Time<<endl;

        covert_ImuData_topic(*imuData,result);
        read_pub.publish(result);
        // cout<<"IMU的时间是" <<buffer[60]<<endl;
        // cout<<"ROS的时间是" <<ros::Time::now().toSec()<<endl;
        return true;
    }
    

    unsigned short GetCheckCode(unsigned char *ptr, unsigned char len)
    {
        unsigned short Code;
        int i,j;

        Code = 0xffff;
        for(i=0;i<len;i++)
        {
            Code ^= ptr[i];
            for(j=0;j<8;j++)
            {
                if(Code&0x0001)
                {
                    Code >>= 1;
                    Code ^= 0xa001;
                }
                else
                {
                    Code >>= 1;
                }
            }
        }
        return (Code);
    }


    // 解析串口为IMU的数据
    // imuData: 用于存储IMU数据的变量
    // buffer: 串口接收IMU具体数据缓冲区
    // length: 缓冲区有效数据的长度
    bool ParseImuData(IMU_DATA_structure* imuData, unsigned char* buffer, int length)
    {
        // unsigned char temp[200];
        // memset(temp,0,200);
        // // cout<<"last_length"<<last_length<<endl;
        // // cout<<"last_buffer"<<(char)last_buffer[0]<<endl;
        // memcpy(last_buffer,temp,last_length);
        // memcpy(buffer,temp+last_length,length);
        // buffer = temp;
        // length+=last_length;
        
        // for(int i=0 ;i<length;i++){
        //     if(buffer[i]!=temp[i])
        //         cout<<"copy error"<<endl;
        // }
   

        static const int DATA_LENGTH = 78;
        if (imuData == NULL || buffer == NULL || length < DATA_LENGTH)
        {
            cout<<"输入错误"<<endl;
            initialize_systeam=false;
            return false;
        }
        int index = 0;     
        
        while (length >= DATA_LENGTH)
        {
            if (buffer[index++] == 0xDA && buffer[index++] == 0x61)
            {
                unsigned short CRC = (uint8_t)buffer[DATA_LENGTH - 2]<<8 | (uint8_t)buffer[DATA_LENGTH - 1];
                // unsigned short crc1 =CRC16_CCITT(buffer,76);
                // unsigned short crc2 = CRC16_CCITT_FALSE(buffer,76);
                // unsigned short crc3 =CRC16_XMODEM(buffer,76);
                // unsigned short crc4 =CRC16_X25(buffer,76);
                // unsigned short crc_check = CRC16_MODBUS(buffer,76);
                // unsigned short crc6 = CRC16_IBM(buffer,76);
                // unsigned short crc7 =CRC16_MAXIM(buffer,76);
                // unsigned short crc8 =CRC16_USB(buffer,76);
                unsigned short crc_check = GetCheckCode(buffer,76);
                // if (crc_check == CRC){
                    ParsePacketB4(imuData, buffer + 2, DATA_LENGTH - 2);
                // }
                index = 0;
                length = length - DATA_LENGTH;
             
                memcpy(buffer, buffer + DATA_LENGTH, length);
                
            }
            else{
                index = 0;
                length = length - 1;
                
                memcpy(buffer, buffer + 1, length);
              
                initialize_systeam=false;
                
            }
        }
        // last_length=length;
        // memset(last_buffer,0,78);
        // if(length!=0){
        //     memcpy(buffer,last_buffer,last_length);
        // }
        // // last_buffer = new(unsigned char);
        // // last_buffer = buffer;
        // cout<<"length"<<length<<endl;
        // // for(int i=0;i<length;i++){
        // //     cout<<"??????????"<<endl;
        // //     cout<<std::showbase<<std::dec<<"buffer: "<<(char )buffer[i]<<endl;// 16
        // // }
        // delete [] temp;
        return true;
    }
};
#endif
