#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <boost/foreach.hpp>
#include <cv_bridge/cv_bridge.h>
#define foreach BOOST_FOREACH

cv::Mat image_get;   // ROS格式转换为CV::MAT格式的图片
double  img_rec_time; // 图片的时间戳

int main(){
    rosbag::Bag bag;
    bag.open("/home/qian/data/lvi_data/degenerate_seq_00.bag", rosbag::bagmode::Read);

    // 需要读取的topic列表
    std::vector<std::string> topics;
    topics.push_back(std::string("/camera/image_color/compressed")); 


    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::CompressedImageConstPtr msg = m.instantiate<sensor_msgs::CompressedImage>();
        try
        {
            cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::RGB8 );
            img_rec_time = msg->header.stamp.toSec();
            image_get = cv_ptr_compressed->image;
            cv_ptr_compressed->image.release();
            cv::imshow("test",image_get);
        }
        catch ( cv_bridge::Exception &e )
        {
            printf( "Could not convert from '%s' to 'bgr8' !!! ", msg->format.c_str() );
        }
        // std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>(); // 这个数据类型
        // if (s != NULL)
        //     std::cout << s->data << std::endl;

        // std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        // if (i != NULL)
        //     std::cout << i->data << std::endl;
    }

    bag.close();
}
