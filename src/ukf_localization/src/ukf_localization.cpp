#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ukf_localization/pose_estimator.hpp>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include "utility.h"
using namespace Eigen;

// TODO euler -> quaternion

class UKFLocalization:public ParamServer{
private:
  ros::NodeHandle nh;
  ros::Subscriber gps_odom_sub;
  ros::Subscriber slam_odom_sub;
  ros::Subscriber imu_body_sub;
  ros::Subscriber imu_world_sub;

  ros::Publisher ukf_odom_pub;
  ros::Publisher pubPath;

  std::mutex mtx;
  nav_msgs::Path globalPath;
  std::deque<nav_msgs::Odometry> gpsQueue; // GPS队列
  std::mutex pose_estimator_mutex;
  std::unique_ptr<ukf_localization::PoseEstimator> pose_estimator;
  bool initialize_systeam=false;
  float cool_time_duration;
  double last_z=0;
  float last_velocity_x=0,last_velocity_y=0,last_velocity_z=0;
  float last_acc_x=0,last_acc_y=0,last_acc_z=0;
  double last_time=0;
  // double gps_qx=0,gps_qy=0,gps_qz=0,gps_qw=1;
  // double slam_qx=0,slam_qy=0,slam_qz=0,slam_qw=1;
  float slam_localization_precision[2]={0};
  double roll, pitch, yaw;//定义存储r\p\y的容器 当前的
  double imu_roll,imu_pitch,imu_yaw;
  double last_imu_roll,last_imu_pitch,last_imu_yaw;
public:
  UKFLocalization():cool_time_duration(1){


      gps_odom_sub= nh.subscribe<nav_msgs::Odometry>("/gps/correct_odom", 1, &UKFLocalization::gpsHandler, this, ros::TransportHints().tcpNoDelay());
      slam_odom_sub= nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry", 1, &UKFLocalization::slamHandler, this, ros::TransportHints().tcpNoDelay());

      ukf_odom_pub = nh.advertise<nav_msgs::Odometry>("ukf/odom", 1);
      pubPath = nh.advertise<nav_msgs::Path>("ukf/path", 1); // 发布路径
  }

    /**
   * @brief convert a Eigen::Matrix to TransformedStamped
   * @param stamp           timestamp
   * @param pose            pose matrix
   * @param frame_id        frame_id
   * @param child_frame_id  child_frame_id
   * @return transform
   */
  geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id) {
    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
    quat.normalize();
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = pose(0, 3);
    odom_trans.transform.translation.y = pose(1, 3);
    odom_trans.transform.translation.z = pose(2, 3);
    odom_trans.transform.rotation = odom_quat;

    return odom_trans;
  }

  void updatePath(const ros::Time& stamp,const Eigen::Matrix4f& pose)
  {
    geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose, "slam_odom","slam_base_link");
    Eigen::Vector3f Pwi(pose(0, 3),pose(1, 3),pose(2, 3));

    // Eigen::Vector3f Pwl;
    // Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
    // quat.normalize();
    // Pwi=Pwl +pose.block<3, 3>(0, 0)*(Pli);
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = stamp;
    pose_stamped.header.frame_id = "slam_odom";
    // pose_stamped.pose.position.x = pose(0, 3);
    // pose_stamped.pose.position.y = pose(1, 3);
    // pose_stamped.pose.position.z = pose(2, 3);
    

    pose_stamped.pose.position.x = Pwi.x();
    pose_stamped.pose.position.y = Pwi.y();
    pose_stamped.pose.position.z =last_z;


    pose_stamped.pose.orientation = odom_trans.transform.rotation;

    globalPath.poses.push_back(pose_stamped);
    if( globalPath.poses.size()>10000)
      globalPath.poses.erase(globalPath.poses.begin());
    // publish path
    if (pubPath.getNumSubscribers() != 0)
    {
        globalPath.header.stamp = stamp;
        globalPath.header.frame_id = "slam_odom";
        pubPath.publish(globalPath);
    }
  }

    /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const nav_msgs::Odometry::ConstPtr& odommsg, const Eigen::Matrix4f& pose) {

    Eigen::Vector3f Pwi(pose(0, 3),pose(1, 3),pose(2, 3));

   

    // publish the transform
    nav_msgs::Odometry odom;
    odom.header.stamp = odommsg->header.stamp;
    odom.header.frame_id = "slam_odom";


    odom.pose.pose.position.x = Pwi.x();
    odom.pose.pose.position.y = Pwi.y();
    odom.pose.pose.position.z =last_z;
    // tf::quaternionMsgToTF(odommsg->pose.pose.orientation, correct_quat);


    tf::Quaternion guihua_error_quaterniond;
    guihua_error_quaterniond.setRPY(roll,pitch,M_PI/2.0+yaw);
    guihua_error_quaterniond.normalize();
    odom.pose.pose.orientation.x = guihua_error_quaterniond.x();
    odom.pose.pose.orientation.y = guihua_error_quaterniond.y();
    odom.pose.pose.orientation.z = guihua_error_quaterniond.z();
    odom.pose.pose.orientation.w = guihua_error_quaterniond.w();
    odom.pose.covariance[0]=slam_localization_precision[0];
    odom.pose.covariance[1]=slam_localization_precision[1];
    odom.twist.twist.linear.x=last_velocity_x;
    odom.twist.twist.linear.y=last_velocity_y;
    odom.twist.twist.linear.z=last_velocity_z;
    odom.twist.twist.angular.x=last_acc_x;
    odom.twist.twist.angular.y=last_acc_y;
    odom.twist.twist.angular.z=last_acc_z;
    ukf_odom_pub.publish(odom);

  }

 
  void slamHandler(const nav_msgs::Odometry::ConstPtr& slamMsg){
     std::lock_guard<std::mutex> lock(mtx);
     slam_localization_precision[0]=slamMsg->pose.covariance[0]; // 角度分数
     slam_localization_precision[1]=slamMsg->pose.covariance[1]; // 面点分数

     // 初始化
    if(!initialize_systeam&&(slamMsg->pose.covariance[0]<0.1)){
      Eigen::Vector3f twl(slamMsg->pose.pose.position.x,slamMsg->pose.pose.position.y,slamMsg->pose.pose.position.z);
      Eigen::Quaternionf qwl(slamMsg->pose.pose.orientation.w,slamMsg->pose.pose.orientation.x,slamMsg->pose.pose.orientation.y,slamMsg->pose.pose.orientation.z);

      Eigen::Vector3f twi;
      twi = qwl*Pli.cast<float>()+ twl;
      twi(0) +=last_velocity_x*0.1;
      twi(1) +=last_velocity_y*0.1;
      Eigen::Quaternionf qwi;
      qwi = qwl*extRot.cast<float>();

      pose_estimator.reset(new ukf_localization::PoseEstimator(
      slamMsg->header.stamp,
      twi,
      qwi,
      cool_time_duration  // imu积分的死区
      ));
      initialize_systeam=true;
      last_z=twi(2);
      last_time=slamMsg->header.stamp.toSec();
      // slam_qx=slamMsg->pose.pose.orientation.x;
      // slam_qy=slamMsg->pose.pose.orientation.y;
      // slam_qz=slamMsg->pose.pose.orientation.z;
      // slam_qw=slamMsg->pose.pose.orientation.w;
      Eigen::Vector3d eulerAngle=qwi.cast<double>().matrix().eulerAngles(0,1,2);
 
      roll = eulerAngle(0);
      pitch = eulerAngle(1);
      yaw = eulerAngle(2);
      last_imu_roll=eulerAngle(0);
      last_imu_pitch=eulerAngle(1);
      last_imu_yaw=eulerAngle(2);
      return;
    }
    if(!initialize_systeam)
      return;


  
    if(slamMsg->pose.covariance[0]<0.2&&slamMsg->pose.covariance[1]<0.2){
      Eigen::Vector3f twl(slamMsg->pose.pose.position.x,slamMsg->pose.pose.position.y,slamMsg->pose.pose.position.z);
      Eigen::Quaternionf qwl(slamMsg->pose.pose.orientation.w,slamMsg->pose.pose.orientation.x,slamMsg->pose.pose.orientation.y,slamMsg->pose.pose.orientation.z);

      Eigen::Vector3f twi;
      twi = qwl*Pli.cast<float>()+ twl;
      twi(0) +=last_velocity_x*0.1;
      twi(1) +=last_velocity_y*0.1;
      Eigen::Quaternionf qwi;
      qwi = qwl*extRot.cast<float>();
      tf::Quaternion slam_quat(qwi.x(),qwi.y(),qwi.z(),qwi.w());
      tf::Matrix3x3(slam_quat).getRPY(roll, pitch, yaw);//进行转换


      pose_estimator->correct(twi(0),twi(1),twi(2),
                          qwi.w(),qwi.x(),qwi.y(),qwi.z());
      
      updatePath(slamMsg->header.stamp,pose_estimator->matrix());
      publish_odometry(slamMsg,pose_estimator->matrix());
      last_z=twi(2);
      last_time=slamMsg->header.stamp.toSec();
      // slam_qx=slamMsg->pose.pose.orientation.x;
      // slam_qy=slamMsg->pose.pose.orientation.y;
      // slam_qz=slamMsg->pose.pose.orientation.z;
      // slam_qw=slamMsg->pose.pose.orientation.w;
 
    }
    // std::cout<<"slam"<<std::endl;
    
  }

  double last_E,last_N,last_U;
  int count=0;
  void gpsHandler(const nav_msgs::Odometry::ConstPtr& gpsMsg){
     std::lock_guard<std::mutex> lock(mtx);
    if(gpsMsg->header.stamp.toSec()<last_time+0.01){
      // std::cout<<"时间不对"<<std::endl;
      return;
    
    }
      
    count++;
    if(!initialize_systeam)
      return;
    count%=5;

    // 下面这个if决定是否使用gps的值
    // if((last_E!=gpsMsg->pose.pose.position.x||last_N!=gpsMsg->pose.pose.position.y)&&count==0){

      // Eigen::Vector3d Pwl;
      // Eigen::Vector3d Pwi(gpsMsg->pose.pose.position.x,gpsMsg->pose.pose.position.y,gpsMsg->pose.pose.position.z);
      // Eigen::Quaterniond Qwi(gpsMsg->pose.pose.orientation.w,gpsMsg->pose.pose.orientation.x,gpsMsg->pose.pose.orientation.y,gpsMsg->pose.pose.orientation.z);
      // Pwl= Pwi+ Qwi.matrix()*Pil;
      // pose_estimator->correct(pose_estimator->matrix()(0,3),pose_estimator->matrix()(1,3),pose_estimator->matrix()(2,3),
      //                     gpsMsg->pose.pose.orientation.w,gpsMsg->pose.pose.orientation.x,gpsMsg->pose.pose.orientation.y,gpsMsg->pose.pose.orientation.z);
      
      // last_E=gpsMsg->pose.pose.position.x;
      // last_N=gpsMsg->pose.pose.position.y;
    // }
    // else{
      pose_estimator->predict(gpsMsg->header.stamp, Eigen::Vector3f(gpsMsg->twist.twist.linear.x, gpsMsg->twist.twist.linear.y, gpsMsg->twist.twist.linear.z), 
        Eigen::Vector3f(0, 0,0));
      last_velocity_x=gpsMsg->twist.twist.linear.x;
      last_velocity_y=gpsMsg->twist.twist.linear.y;
      last_velocity_z=gpsMsg->twist.twist.linear.z;
      last_acc_x = gpsMsg->twist.twist.angular.x;
      last_acc_y = gpsMsg->twist.twist.angular.y;
      last_acc_z = gpsMsg->twist.twist.angular.z;
      // gps_qx=gpsMsg->pose.pose.orientation.x;
      // gps_qy=gpsMsg->pose.pose.orientation.y;
      // gps_qz=gpsMsg->pose.pose.orientation.z;
      // gps_qw=gpsMsg->pose.pose.orientation.w;
    // }

  
    // slam_qx=slamMsg->pose.pose.orientation.x;
    // slam_qy=slamMsg->pose.pose.orientation.y;
    // slam_qz=slamMsg->pose.pose.orientation.z;
    // slam_qw=slamMsg->pose.pose.orientation.w;
    tf::Quaternion imu_quat(gpsMsg->pose.pose.orientation.x,gpsMsg->pose.pose.orientation.y,gpsMsg->pose.pose.orientation.z,gpsMsg->pose.pose.orientation.w);
    tf::Matrix3x3(imu_quat).getRPY(imu_roll, imu_pitch, imu_yaw);//进行转换
    roll+=(imu_roll-last_imu_roll);
    pitch+=(imu_pitch-last_imu_pitch);
    yaw+=(imu_yaw-last_imu_yaw);
    
    if(count==0)
      updatePath(gpsMsg->header.stamp,pose_estimator->matrix());
    publish_odometry(gpsMsg,pose_estimator->matrix());
    last_imu_roll=imu_roll;
    last_imu_pitch=imu_pitch;
    last_imu_yaw=imu_yaw;

 
  }
};


int main (int argc, char** argv)
{
  //初始化节点
  ros::init(argc, argv, "ukf_localization_node");

  UKFLocalization gps_lio_sam;

  ROS_INFO("\033[1;32m----> UKF localization Started.\033[0m");


  





  ros::spin();
  return 0;
 
}
