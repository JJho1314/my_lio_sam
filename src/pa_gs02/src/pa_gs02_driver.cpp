/****************************************
*    @author  : Qian Chenglong
*    @date    : 20201005
*    @ROS     : Melodic
*    @Version : 1.0.0
****************************************/

#include <pa_gs02_driver.h>



int main (int argc, char** argv)
{
  //初始化节点
  ros::init(argc, argv, "serial_imu_node");

  Parse_GPS pa_gs02;

  ROS_INFO("\033[1;32m----> Obtain GPS Started.\033[0m");

  return 0;
 
}
