#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
ros::Publisher imu_pub;
ros::Subscriber imu_sub;
void imucallback(const sensor_msgs::ImuConstPtr imu_in){
    // std::cout<<"imu"<<std::endl;
    sensor_msgs::Imu imu_msg;
    imu_msg=*imu_in;
    Eigen::Quaterniond quaternion(imu_in->orientation.w,imu_in->orientation.x,imu_in->orientation.y,imu_in->orientation.z);
    Eigen::Vector3d eulerAngle=quaternion.matrix().eulerAngles(2,1,0);
    std::cout<<"imu"<<eulerAngle[0]<<","<<eulerAngle[1]<<","<<eulerAngle[2]<<std::endl;
    eulerAngle[0]+=M_PI*2.9607077/180.0;
       
       
    while(eulerAngle[0] > M_PI){
        eulerAngle[0] = eulerAngle[0] -2*M_PI;
    }
    while(eulerAngle[0]<- M_PI){
        eulerAngle[0] = eulerAngle[0] + 2*M_PI;
    }
    tf::Quaternion imu_quaterniond;
    imu_quaterniond.setRPY(eulerAngle[2],eulerAngle[1],eulerAngle[0]);
    imu_msg.orientation.w=imu_quaterniond.w();
    imu_msg.orientation.x=imu_quaterniond.x();
    imu_msg.orientation.y=imu_quaterniond.y();
    imu_msg.orientation.z=imu_quaterniond.z();
    imu_pub.publish(imu_msg);
}
int main (int argc, char** argv)
{
    
  
  //初始化节点
  ros::init(argc, argv, "imu_convert_node");

  ros::NodeHandle nh;
  imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/correct", 10);
  imu_sub= nh.subscribe<sensor_msgs::Imu>  ("ins550d/imu/data",10,imucallback);


  ROS_INFO("\033[1;32m----> Obtain IMU Correct Started.\033[0m");
  ros::spin();
  return 0;
 
}

