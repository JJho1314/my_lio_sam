#include <string>
#include <ros/ros.h>                           // 包含ROS的头文件
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <math.h>
#include "std_msgs/String.h"              //ros定义的String数据类型

using namespace std;
using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作

unsigned char buf[78];                      //定义字符串长度

int main(int argc, char** argv) {

    ros::init(argc, argv, "boost");       //初始化节点
    ros::NodeHandle n;
    
   ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);      //定义发布消息的名称及sulv

  ros::Rate loop_rate(400);


    io_service iosev;
    serial_port sp(iosev, "/dev/ttyUSB0");         //定义传输的串口
    sp.set_option(serial_port::baud_rate(460800));   
    sp.set_option(serial_port::flow_control());
    sp.set_option(serial_port::parity());
    sp.set_option(serial_port::stop_bits());
    sp.set_option(serial_port::character_size(8));

    while (ros::ok()) {
      // write(sp, buffer(buf1, 6));  //write the speed for cmd_val    
     //write(sp, buffer("Hello world", 12));  
     read (sp,buffer(buf));
     string str(&buf[0],&buf[22]);              //将数组转化为字符串
  //if(buf[0]=='p' && buf[21] == 'a')
   // {
       std_msgs::String msg;
       std::stringstream ss;
       ss <<str;
     
      msg.data = ss.str();
     ROS_INFO_STREAM("Read: " << msg.data.length());

    chatter_pub.publish(msg);   //发布消息

    ros::spinOnce();

    loop_rate.sleep();
  //  }
    }

    iosev.run(); 
    return 0;
}