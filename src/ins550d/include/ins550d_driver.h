#ifndef PA_GS02_DRIVER_H
#define PA_GS02_DRIVER_H

#include<fstream>  
#include<iostream>  
#include<string> 

#include <ros/ros.h>
#include <sys/time.h>
#include<chrono>
#include <sensor_msgs/Imu.h>
// #include <boost/asio.hpp>        
// #include <boost/bind.hpp>
#include <tf/tf.h>
#include <serial/serial.h>  
#include <Daoyuan.h>
#include <std_msgs/Empty.h>

#include <std_msgs/String.h>
#include <eigen3/Eigen/Dense>
#include <crc16.h>

#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#define G 9.80665
#define ellipse_a 6378137
#define ellipse_e 0.081819190842622
#pragma pack(1)
using namespace std;  

int last_length=0;
int mycount =0;
class Parse_GPS
{
private:

  
    std::string inFilePath = "./";
    std::string outFilePath = "./";
    ros::NodeHandle nh;
    ros::NodeHandle nh_get;
    ros::Publisher read_pub;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/ins550d/imu/data", 10);
    ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 10);
    ros::Publisher gpsOdom_pub = nh.advertise<nav_msgs::Odometry>("/discovery/location/pose", 10);
    ros::Publisher gpsCorrect_Odom_pub = nh.advertise<nav_msgs::Odometry>("/gps/correct_odom", 10);
    pub_imu::Daoyuan daoyuan_msg;
    ros::Publisher daoyuan_pub = nh.advertise<pub_imu::Daoyuan>("/daoyuan_data", 10);
    serial::Serial ser_IMU; //声明串口对象
    string imuport;  // imu端口名
    std::chrono::steady_clock::time_point now;
    std::chrono::steady_clock::time_point last;

    const float MinLength = 58;
    double daoyuan_roll,daoyuan_pitch,daoyuan_yaw;
    struct IMU_DATA_structure
    {
        uint8_t head[3];
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
        int16_t wx;
        int16_t wy;
        int16_t wz;
        int16_t ax;
        int16_t ay;
        int16_t az;
        int32_t latitude;
        int32_t lontitude;
        int32_t height;
        int16_t vnorth;  // 北东地方向的速度
        int16_t veast;
        int16_t vheight;
        uint8_t status;
        uint8_t nouse[6];  // 精度轮询
        int16_t data[3];   // 设备状态
        uint32_t time;     // 时间
        uint8_t type;
        uint8_t check;
    };

    double BaseMatrix[4][3];
    bool initialize_systeam=false;
    sensor_msgs::Imu imu_msg;
    struct IMU_DATA_structure daoyuan_stc;

public:

    unsigned char* last_buffer;
    
    
    Parse_GPS():nh_get("~"),last_buffer(NULL)
    {
        //read_pub = nh.advertise<sensor_msgs::Imu>("/imu/data",1);//串口接收  //发布数据
        
        nh_get.getParam("imuport",imuport);
        try
        {
     
            ser_IMU.setPort(imuport);//串口名称
            ser_IMU.setBaudrate(230400);//波特率
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser_IMU.setTimeout(to);
            ser_IMU.open();
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR_STREAM("Unable to open port ");
            exit(0);
        }
  
        if(ser_IMU.isOpen())
        {
            ROS_INFO_STREAM("Serial Port initialized");
        }
        else
        {
            exit(0);
        }

    double base_point_lat=nh.param<double>("base_point_lat",34.2569999);
    double base_point_lon=nh.param<double>("base_point_lon",108.6511768);
    double base_point_hei=nh.param<double>("base_point_hei",392.931);


    // ROI
    // double base_point_lat=nh.param<double>("base_point_lat",34.2640159);
    // double base_point_lon=nh.param<double>("base_point_lon",108.6559782);
    // double base_point_hei=nh.param<double>("base_point_hei",392.8);
    // double base_point_lat=nh.param<double>("base_point_lat",34.2570016);
    // double base_point_lon=nh.param<double>("base_point_lon",108.6511872);
    // double base_point_hei=nh.param<double>("base_point_hei",392.931);
    //std::string inFilePath = nh.param<std::string>("InFilePath","null");
    //std::string outFilePath = nh.param<std::string>("OutFilePath","null");

    
    // 计算基础矩阵
    GenerateConvertMatrix(BaseMatrix,base_point_lat,base_point_lon,base_point_hei);

        ros::Rate loop_rate(250);//指定循环的频率 
        while(ros::ok())
        {
            now = std::chrono::steady_clock::now();
            
            double t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(now - last).count();
            last = now;
            //  cout<<"一个周期的时间是"<<t_track<<endl;


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
                    while(*temp!=0xBD);
                    imu_str.data = ser_IMU.read(1);
                    temp=(unsigned char *)imu_str.data.c_str();
                    if(*temp==0xDB){
                        imu_str.data = ser_IMU.read(1);
                        temp=(unsigned char *)imu_str.data.c_str();
                        if(*temp==0x0B){
                            initialize_systeam=true;
                            ser_IMU.read(55); //58阵-3（针头）
                            cout<<"systeam initializes successful"<<endl;
                        }
                    }
                    
                 
                }
                else
                {
                
                //  cout<<"systeam ready"<<endl;
                    imu_str.data = ser_IMU.read(58); 
                    ParseImuData(&daoyuan_stc,(unsigned char *)imu_str.data.c_str(),imu_str.data.length());
               
                     
                }
                
               
            }

            //处理ROS的信息，比如订阅消息,并调用回调函数
            ros::spinOnce();
            loop_rate.sleep();

        }
    }
 
    const double nature_e=2.718281;
    double precision_lat=10,precision_long=10,precision_height=100;
    double precision_R=10,precision_P,precision_Y;

    double last_wx,last_wy,last_wz;
    // 参数说明
    // imuData: 用于存储IMU数据的变量
    // buffer: IMU具体数据缓冲区
    // length: 这次解析数据的长度
    bool ParsePacketB4(IMU_DATA_structure* daoyuan_stc, unsigned char* buffer, int length)
    {
      
             
        if (daoyuan_stc == NULL || buffer == NULL || length != MinLength)
        {
            initialize_systeam=false;
            cout<<"输入错误"<<endl;
            return false;
        }


        memcpy(daoyuan_stc,buffer,length);

        imu_msg.header.stamp=ros::Time::now();
        daoyuan_msg.header.stamp=imu_msg.header.stamp;
        imu_msg.header.frame_id = "slam_base_link";
        imu_msg.linear_acceleration.x=(float)daoyuan_stc->ay*12.0/32768.0*G;
        imu_msg.linear_acceleration.y=(float)daoyuan_stc->ax*12.0/32768.0*G;
        imu_msg.linear_acceleration.z=-(float)daoyuan_stc->az*12.0/32768.0*G;


        imu_msg.angular_velocity.x=(float)daoyuan_stc->wy*300.0/32768.0/180.0*M_PI;
        imu_msg.angular_velocity.y=(float)daoyuan_stc->wx*300.0/32768.0/180.0*M_PI;
        imu_msg.angular_velocity.z=-(float)daoyuan_stc->wz*300.0/32768.0/180.0*M_PI;
        if(abs(imu_msg.angular_velocity.x)>M_PI/2){
            imu_msg.angular_velocity.x=last_wx;
        }
        if(abs(imu_msg.angular_velocity.y)>M_PI/2){
            imu_msg.angular_velocity.y=last_wy;
        }
        if(abs(imu_msg.angular_velocity.z)>M_PI/2){
            imu_msg.angular_velocity.z=last_wz;
        }
        last_wx=imu_msg.angular_velocity.x;
        last_wy=imu_msg.angular_velocity.y;
        last_wz=imu_msg.angular_velocity.z;
        double daoyuan_roll,daoyuan_pitch,daoyuan_yaw;
        daoyuan_roll=(double)daoyuan_stc->pitch*M_PI/16384.0;
        daoyuan_pitch=(double)daoyuan_stc->roll*M_PI/16384.0; // 弧度
        daoyuan_yaw =-(double)daoyuan_stc->yaw*M_PI/16384.0;
        // daoyuan_yaw +=M_PI*1.25/180.0;
        // daoyuan_yaw =0;
       
        while(daoyuan_yaw > M_PI){
            daoyuan_yaw = daoyuan_yaw -2*M_PI;
        }
        while(daoyuan_yaw<- M_PI){
            daoyuan_yaw = daoyuan_yaw + 2*M_PI;
        }
        while(daoyuan_pitch > M_PI){
            daoyuan_pitch = daoyuan_pitch - 2*M_PI;
        }
        while(daoyuan_pitch<-M_PI){
            daoyuan_pitch = daoyuan_pitch + 2*M_PI;
        }
        while(daoyuan_roll > M_PI){
            daoyuan_roll = daoyuan_roll - 2*M_PI;
        }
        while(daoyuan_roll<-M_PI){
            daoyuan_roll = daoyuan_roll + 2*M_PI;
        }
        std::cout<<"RPY is"<<daoyuan_roll/M_PI*180.f<<","<<daoyuan_pitch/M_PI*180.f<<","<<daoyuan_yaw/M_PI*180.f<<std::endl;
        tf::Quaternion imu_quaterniond;
        
        imu_quaterniond.setRPY(daoyuan_roll,daoyuan_pitch,daoyuan_yaw);
        imu_msg.orientation.w=imu_quaterniond.w();
        imu_msg.orientation.x=imu_quaterniond.x();
        imu_msg.orientation.y=imu_quaterniond.y();
        imu_msg.orientation.z=imu_quaterniond.z();
        imu_pub.publish(imu_msg);

        sensor_msgs::NavSatFix gps_msg;
        gps_msg.header.stamp=imu_msg.header.stamp;
        gps_msg.header.frame_id = "navsat_link";
        gps_msg.latitude=(float)daoyuan_stc->latitude*1e-7;
        gps_msg.longitude=(float)daoyuan_stc->lontitude*1e-7;
        gps_msg.altitude=(float)daoyuan_stc->height*1e-3;
        if(daoyuan_stc->type==0){
            precision_lat=powf(nature_e,daoyuan_stc->data[0]/100.f);
            precision_lat*=precision_lat;
            precision_long=powf(nature_e,daoyuan_stc->data[1]/100.f);
            precision_long*=precision_long;
            precision_height=powf(nature_e,daoyuan_stc->data[2]/100.f)*10;
            // precision_height*=precision_height;
            
        }
        else if(daoyuan_stc->type==2){
            precision_R=powf(nature_e,daoyuan_stc->data[0]/100.f);
            // precision_R*=precision_R;
            precision_P=powf(nature_e,daoyuan_stc->data[1]/100.f);
            // precision_P*=precision_P;
            precision_Y=powf(nature_e,daoyuan_stc->data[2]/100.f);
            // precision_Y*=precision_Y;
        }
     

        // 原始数据，连单位都没转换 ,
        daoyuan_msg.roll=daoyuan_stc->roll;
        daoyuan_msg.pitch=daoyuan_stc->pitch;
        daoyuan_msg.yaw=daoyuan_stc->yaw;
        daoyuan_msg.wx=daoyuan_stc->wx;
        daoyuan_msg.wy=daoyuan_stc->wy;
        daoyuan_msg.wz=daoyuan_stc->wz;
        daoyuan_msg.ax=daoyuan_stc->ax;
        daoyuan_msg.ay=daoyuan_stc->ay;
        daoyuan_msg.az=daoyuan_stc->az;
        daoyuan_msg.lat=daoyuan_stc->latitude;
        daoyuan_msg.lon=daoyuan_stc->lontitude;
        daoyuan_msg.hei=daoyuan_stc->height;
        daoyuan_msg.vx=daoyuan_stc->vnorth;
        daoyuan_msg.vy=daoyuan_stc->veast;
        daoyuan_msg.vz=daoyuan_stc->vheight;
        daoyuan_msg.status=daoyuan_stc->status;
        daoyuan_msg.data1=daoyuan_stc->data[0];
        daoyuan_msg.data2=daoyuan_stc->data[1];
        daoyuan_msg.data3=daoyuan_stc->data[2];
        daoyuan_msg.imu_time=daoyuan_stc->time;
        daoyuan_msg.type=daoyuan_stc->type;
        // cout<<"当前类型为"<<(int)daoyuan_stc->type<<endl;
        double lat = (double)daoyuan_msg.lat*1e-7;
        double lon = (double)daoyuan_msg.lon*1e-7;
        double hei = (double)daoyuan_msg.hei*1e-3;
        double x,y,z;
        // double roll = (double) daoyuan_msg.roll/16384.0*M_PI;
        // double pitch= (double) daoyuan_msg.pitch/16384.0*M_PI;
        // double yaw  = (double) daoyuan_msg.yaw/16384.0*M_PI;
        cout <<setprecision(12)<<"lla:" <<lat << " ," <<lon << " ," <<hei << endl;
        PointData2ENU(lat,lon,hei,x,y,z,BaseMatrix);

        cout <<"xyz:" <<x << " ," << y << " ," << z << endl;
        

        // generateHdlLocationLaunchFile(x,y,z,roll,pitch,yaw,inFilePath,outFilePath);
        
        // Eigen::Quaterniond quaternion(imu_msg.orientation.w,imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z);
        // Eigen::Vector3d acc(imu_msg.linear_acceleration.x,imu_msg.linear_acceleration.y,imu_msg.linear_acceleration.z);
       
        // acc=quaternion*acc;
        nav_msgs::Odometry gps_odom;
        gps_odom.header.stamp=imu_msg.header.stamp;
        gps_odom.header.frame_id = "slam_base_link";
        gps_odom.pose.pose.position.x=x;
        gps_odom.pose.pose.position.y=y;
        gps_odom.pose.pose.position.z=z;
        gps_odom.pose.covariance[0]=precision_lat;
        gps_odom.pose.covariance[7]=precision_long;
        // gps_odom.pose.pose.position.x=110;
        // gps_odom.pose.pose.position.y=-110;
        // gps_odom.pose.pose.position.z=10;
        tf::Quaternion gps_odom_quaterniond;
        gps_odom_quaterniond.setRPY(daoyuan_roll,daoyuan_pitch,M_PI/2.0+daoyuan_yaw);
        gps_odom_quaterniond.normalize();
        gps_odom.pose.pose.orientation.w=gps_odom_quaterniond.w();
        gps_odom.pose.pose.orientation.x=gps_odom_quaterniond.x();
        gps_odom.pose.pose.orientation.y=gps_odom_quaterniond.y();
        gps_odom.pose.pose.orientation.z=gps_odom_quaterniond.z();

        gps_odom.twist.twist.linear.x =daoyuan_stc->veast*1e2/32768.0;
        gps_odom.twist.twist.linear.y =daoyuan_stc->vnorth*1e2/32768.0;
        gps_odom.twist.twist.linear.z =-daoyuan_stc->vheight*1e2/32768.0;


        gps_odom.twist.twist.angular.x =imu_msg.linear_acceleration.x;
        gps_odom.twist.twist.angular.y =imu_msg.linear_acceleration.y;
        gps_odom.twist.twist.angular.z =imu_msg.linear_acceleration.z;
   
        gpsOdom_pub.publish(gps_odom);
 
        nav_msgs::Odometry gps_Correctodom;
        gps_Correctodom.header.stamp=imu_msg.header.stamp;
        gps_Correctodom.header.frame_id = "slam_base_link";
        gps_Correctodom.pose.pose.position.x=x;
        gps_Correctodom.pose.pose.position.y=y;
        gps_Correctodom.pose.pose.position.z=z;
        gps_Correctodom.pose.covariance[0]=precision_lat;
        gps_Correctodom.pose.covariance[7]=precision_long;
        gps_Correctodom.pose.pose.orientation.w=imu_msg.orientation.w;
        gps_Correctodom.pose.pose.orientation.x=imu_msg.orientation.x;
        gps_Correctodom.pose.pose.orientation.y=imu_msg.orientation.y;
        gps_Correctodom.pose.pose.orientation.z=imu_msg.orientation.z;

        gps_Correctodom.twist.twist.linear.x =daoyuan_stc->veast*1e2/32768.0;
        gps_Correctodom.twist.twist.linear.y =daoyuan_stc->vnorth*1e2/32768.0;
        gps_Correctodom.twist.twist.linear.z =-daoyuan_stc->vheight*1e2/32768.0;


        gps_Correctodom.twist.twist.angular.x =imu_msg.linear_acceleration.x;
        gps_Correctodom.twist.twist.angular.y =imu_msg.linear_acceleration.y;
        gps_Correctodom.twist.twist.angular.z =imu_msg.linear_acceleration.z;
        gpsCorrect_Odom_pub.publish(gps_Correctodom);
     

        if (gps_pub.getNumSubscribers() != 0)
            gps_pub.publish(gps_msg);
        if (daoyuan_pub.getNumSubscribers() != 0)
            daoyuan_pub.publish(daoyuan_msg);
        // ROS_INFO_STREAM("pub imu success");
        return true;
    }

    // 解析串口为IMU的数据
    // daoyuan_stc: 用于存储IMU数据的变量
    // buffer: 串口接收IMU具体数据缓冲区
    // length: 缓冲区有效数据的长度
    bool ParseImuData(IMU_DATA_structure* daoyuan_stc, unsigned char* buffer, int length)
    {

        static const int DATA_LENGTH = 58;
        if (daoyuan_stc == NULL || buffer == NULL || length < DATA_LENGTH)
        {
            
            cout<<length<<endl;
            initialize_systeam=false;
            return false;
        }
        int index = 0;     
        
        while (length >= DATA_LENGTH)
        {
            if (buffer[index++] == 0xBD && buffer[index++] == 0xDB && buffer[index++] == 0x0B)
            {
     
                if (checkImuData(buffer)){
                    
                    ParsePacketB4(daoyuan_stc, buffer , DATA_LENGTH );
                }
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
        return true;
    }


    bool checkImuData(unsigned char *imu_data){
        uint8_t check  =(uint8_t)imu_data[57];
        uint8_t value  =0x6D;
        for(int i=3;i<57;i++){
            value^=(uint8_t)imu_data[i];
        }
        if(value==check){
          return true;
     }
     return false;
    }
    void PointData2ENU(double latitude, double longitude, double height, double &outputx, double &outputy, double &outputz, double ConvertMatrix[4][3])
{
	
	double N, x1, y1, z1, dx, dy, dz, x0, y0, z0;
	x0 = ConvertMatrix[3][0]; y0 = ConvertMatrix[3][1]; z0 = ConvertMatrix[3][2];

	latitude = latitude / 180.0 * M_PI;
	longitude = longitude / 180.0 * M_PI;

	double sin_long=sin(longitude);
    double cos_long=cos(longitude);
    double sin_lat =sin(latitude);
    double cos_lat =cos(latitude);

	N = ellipse_a / (sqrt(1 - ellipse_e*ellipse_e*sin_lat*sin_lat));
	x1 = (N + height)*cos_lat*cos_long;
	y1 = (N + height)*cos_lat*sin_long;
	z1 = (N*(1 - ellipse_e*ellipse_e) + height)*sin_lat;
	dx = x1 - x0; dy = y1 - y0; dz = z1 - z0;
	outputy = ConvertMatrix[0][0] * dx + ConvertMatrix[0][1] * dy + ConvertMatrix[0][2] * dz;
	outputx = ConvertMatrix[1][0] * dx + ConvertMatrix[1][1] * dy;
	outputz = ConvertMatrix[2][0] * dx + ConvertMatrix[2][1] * dy + ConvertMatrix[2][2] * dz;

}

void generateHdlLocationLaunchFile(double x,double y,double z,double roll,double pitch,double yaw,std::string inFilePath,std::string outFilePath){
	Eigen::Vector3d eulerAngle(-yaw,-pitch,-roll);
	Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quaterniond;
    quaterniond=yawAngle*pitchAngle*rollAngle;
    
    std::ifstream InFile(inFilePath);
	std::ofstream OutFile(outFilePath);
	
    std::string readLine;
    if(InFile){
        while(std::getline(InFile,readLine)){
            if (readLine.find("init_pos_x")!=std::string::npos)
            {
                OutFile<<"\t\t<param name=\"init_pos_x\" value=\""+std::to_string(x)+"\"/>"<<std::endl;
                OutFile<<"\t\t<param name=\"init_pos_y\" value=\""+std::to_string(y)+"\"/>"<<std::endl;
                OutFile<<"\t\t<param name=\"init_pos_z\" value=\""+std::to_string(z)+"\"/>"<<std::endl;
	            OutFile<<"\t\t<param name=\"init_ori_w\" value=\""+std::to_string(quaterniond.w())+"\"/>"<<std::endl;
	            OutFile<<"\t\t<param name=\"init_ori_x\" value=\""+std::to_string(quaterniond.x())+"\"/>"<<std::endl;
	            OutFile<<"\t\t<param name=\"init_ori_y\" value=\""+std::to_string(quaterniond.y())+"\"/>"<<std::endl;
	            OutFile<<"\t\t<param name=\"init_ori_z\" value=\""+std::to_string(quaterniond.z())+"\"/>"<<std::endl;
                for(int tempI=0;tempI<6;tempI++){
                    std::getline(InFile,readLine);
                }
            }else{
                OutFile<<readLine<<std::endl;
                OutFile.flush();
            }
            
        }
    }
    InFile.close();
    OutFile.close();
}

void GenerateConvertMatrix(double ConvertMatrix[4][3], double latitude, double longitude, double height){
    double N;

	latitude = latitude / 180.0 *M_PI ;
	longitude = longitude / 180.0 * M_PI;
    double sin_long=sin(longitude);
    double cos_long=cos(longitude);
    double sin_lat =sin(latitude);
    double cos_lat =cos(latitude);

	N = ellipse_a / (sqrt(1 - ellipse_e*ellipse_e*sin_lat*sin_lat));
   
	ConvertMatrix[1][0] = -sin_long;
	ConvertMatrix[1][1] = cos_long;
	ConvertMatrix[1][2] = 0;
    ConvertMatrix[0][0] = -sin_lat*cos_long;
	ConvertMatrix[0][1] = -sin_lat*sin_long;
	ConvertMatrix[0][2] = cos_lat;
	ConvertMatrix[2][0] = cos_lat*cos_long;
	ConvertMatrix[2][1] = cos_lat*sin_long;
	ConvertMatrix[2][2] = sin_lat;

	ConvertMatrix[3][0] = (N + height)*cos_lat*cos_long;
	ConvertMatrix[3][1] = (N + height)*cos_lat*sin_long;
	ConvertMatrix[3][2] = (N*(1 - ellipse_e*ellipse_e) + height)*sin_lat;
}

};
#endif