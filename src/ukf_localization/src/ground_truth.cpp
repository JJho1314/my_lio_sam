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
#include <evo/save_trajectory_tum.h>
#include "utility.h"
using namespace Eigen;


vector<Trajectory> trajectory;
vector<Trajectory> trajectory_slam;
class GroundTruth:public ParamServer{
private:
  ros::NodeHandle nh;
  ros::Subscriber gps_odom_sub;
  ros::Subscriber slam_odom_sub;

  ros::Publisher pubPath;

  nav_msgs::Path globalPath;

public:
  GroundTruth(){


      gps_odom_sub= nh.subscribe<nav_msgs::Odometry>("/gps/correct_odom", 1, &GroundTruth::gpsHandler, this, ros::TransportHints().tcpNoDelay());
      slam_odom_sub= nh.subscribe<nav_msgs::Odometry>("ukf/odom", 1, &GroundTruth::slamHandler, this, ros::TransportHints().tcpNoDelay());
   
      pubPath = nh.advertise<nav_msgs::Path>("ground_truth/path", 1); // 发布路径
  }

    void slamHandler(const nav_msgs::Odometry::ConstPtr& slamMsg){
 



    Eigen::Vector3d Pwi;
    Eigen::Vector3d Pli(-Pil.x(),-Pil.y(),-Pil.z());

    Eigen::Vector3d Pwl(slamMsg->pose.pose.position.x,slamMsg->pose.pose.position.y,slamMsg->pose.pose.position.z);
    Eigen::Quaterniond Qwl(slamMsg->pose.pose.orientation.w,slamMsg->pose.pose.orientation.x,slamMsg->pose.pose.orientation.y,slamMsg->pose.pose.orientation.z);

    Pwi= Pwl+Qwl*(Pli);

    savePath(slamMsg->header.stamp,Pwi.x(),Pwi.y(),0,
    slamMsg->pose.pose.orientation.x,slamMsg->pose.pose.orientation.y,slamMsg->pose.pose.orientation.z,slamMsg->pose.pose.orientation.w,trajectory_slam);
 
  }


  void gpsHandler(const nav_msgs::Odometry::ConstPtr& gpsMsg){
 

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = gpsMsg->header.stamp;
    pose_stamped.header.frame_id = "slam_odom";
    pose_stamped.pose.position.x = gpsMsg->pose.pose.position.x;
    pose_stamped.pose.position.y = gpsMsg->pose.pose.position.y;
    pose_stamped.pose.position.z = gpsMsg->pose.pose.position.z;
    pose_stamped.pose.orientation = gpsMsg->pose.pose.orientation;

    globalPath.poses.push_back(pose_stamped);
    if( globalPath.poses.size()>10000)
      globalPath.poses.erase(globalPath.poses.begin());
    savePath(gpsMsg->header.stamp,gpsMsg->pose.pose.position.x,gpsMsg->pose.pose.position.y,0,
    gpsMsg->pose.pose.orientation.x,gpsMsg->pose.pose.orientation.y,gpsMsg->pose.pose.orientation.z,gpsMsg->pose.pose.orientation.w,trajectory);
    // publish path
    if (pubPath.getNumSubscribers() != 0)
    {
        globalPath.header.stamp = gpsMsg->header.stamp;
        globalPath.header.frame_id = "slam_odom";
        pubPath.publish(globalPath);
    }
  }
};


int main (int argc, char** argv)
{
  //初始化节点
  ros::init(argc, argv, "ukf_localization_node");

  GroundTruth Ground_truth;

  ROS_INFO("\033[1;32m----> Drawing Ground_truth Started.\033[0m");
  // double yaw=(-81.39-(-79.68))/180.0*M_PI;
  // double P=(-0.93-(-0.23))/180.0*M_PI;
  // double R=(0.46-(-0.84))/180.0*M_PI;
  // std::cout<<"RPY"<<R<<","<<P<<","<<yaw<<std::endl;
  // Eigen::AngleAxisd rollAngle(AngleAxisd(R,Vector3d::UnitX()));
  // Eigen::AngleAxisd pitchAngle(AngleAxisd(P,Vector3d::UnitY()));
  // Eigen::AngleAxisd yawAngle(AngleAxisd(yaw,Vector3d::UnitZ()));
  
  // Eigen::Matrix3d rotation_matrix;
  // rotation_matrix=yawAngle*pitchAngle*rollAngle;

  // std::cout<<rotation_matrix<<std::endl;



  // Eigen::Matrix3d rotation_matrix;
  // rotation_matrix<< 1.000,	-0.001,	-0.002,
  //                   0.001,	1.000,	0.004,
  //                   0.002,	-0.004,	1.000;
  // Eigen::Vector3d eulerAngle=rotation_matrix.eulerAngles(2,1,0);
  // std::cout<<"YPR"<<eulerAngle[0]<<eulerAngle[1]<<eulerAngle[2]<<std::endl;

  ros::spin();

  SaveTrajectoryTUM("/home/iairiv/data/gps.txt",trajectory);
  SaveTrajectoryTUM("/home/iairiv/data/slam.txt",trajectory_slam);
  return 0;
 
}
