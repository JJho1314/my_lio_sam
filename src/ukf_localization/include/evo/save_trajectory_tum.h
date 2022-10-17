#ifndef SAVE_TRAJECTORY_TUM_H
#define SAVE_TRAJECTORY_TUM_H
#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <chrono>
using namespace std;
struct Trajectory{
    ros::Time time;
    // Position
    double x;
    double y;
    double z;
    // orientation
    double q_x;
    double q_y;
    double q_z;
    double q_w;
};



void savePath(ros::Time time,double x,double y,double z,double q_x,double q_y,double q_z,double q_w,vector<Trajectory> &trajectory){
   struct Trajectory temp;
   temp.time=time;
   temp.x=x;
   temp.y=y;
   temp.z=z;
   temp.q_x=q_x;
   temp.q_y=q_y;
   temp.q_z=q_z;
   temp.q_w=q_w;
   trajectory.emplace_back(temp);

}

void SaveTrajectoryTUM(const string &filename,const vector<Trajectory> &trajectory) {
    while (ros::ok()){
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    ofstream f;
    f.open(filename.c_str());
    f << fixed;
    for (size_t i = 0; i < trajectory.size(); i++) {
        f << setprecision(6) << trajectory.at(i).time << setprecision(7) << " " << trajectory.at(i).x << " "
          << trajectory.at(i).y << " " << trajectory.at(i).z
          << " " << trajectory.at(i).q_x << " " << trajectory.at(i).q_y << " " << trajectory.at(i).q_z << " "
          << trajectory.at(i).q_w << endl;
    }
    f.close();
}

#endif