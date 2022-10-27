// 点云去畸变，去无效点（无穷大的点），融合imu的数据生成cloud_info

#include "utility.h"
#include "lio_sam/cloud_info.h"
#include <pcl/filters/passthrough.h>
// Velodyne点云点结构体构造，point4d是xyz和强度intensity.ring是线数，EIGEN_MAKE_ALIGNED_OPERATOR_NEW字符对齐
#if 0
struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

#endif

#if 1
// rslidar的点云格式
struct PointXYZIRT
{
    PCL_ADD_POINT4D;
    uint8_t intensity;
    uint16_t ring = 0;
    double timestamp = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))
#else

// rslidar的语义点云格式
struct PointXYZIRT
{
    PCL_ADD_POINT4D;
    float intensity;
    float ring = 0;
    float timestamp = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, ring, ring)(float, timestamp, timestamp))
#endif
// Ouster
// struct PointXYZIRT {
//     PCL_ADD_POINT4D;
//     float intensity;
//     uint32_t t;
//     uint16_t reflectivity;
//     uint8_t ring;
//     uint16_t noise;
//     uint32_t range;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// }EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
//     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
//     (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
//     (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
// )

// const int queueLength =500;// IMU队列长度 ,应该是和imu的频率一致

class ImageProjection : public ParamServer
{
private:
    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher pubLaserCloud;

    const int *queueLength = new int(imuFrequency); // IMU队列长度
    //输出距离不过近的点云点
    ros::Publisher pubExtractedCloud;
    //输出下文的cloudInfo
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    // 两scan间的imu角速度积分出来的imu的转角队列
    // TODO为什么不用imu直接输出的角度替换角速度积分的结果呢？这样更准确，是因为积分时间很短吗？
    // 因为是为了得到一个增量而不是绝对量 // TODO 增量也可用用两次做差吧
    double *imuTime = new double[*queueLength];
    double *imuRotX = new double[*queueLength];
    double *imuRotY = new double[*queueLength];
    double *imuRotZ = new double[*queueLength];

    int imuPointerCur;                 // 从当前scan开始的imu积分点数目
    int OdomPointerCur;                // 从当前scan开始的imu积分点数目
    bool firstPointFlag;               // scan的第一个点 falg
    Eigen::Affine3f transStartInverse; //第一个点的transform.inverse()

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn; // 当前真在被处理的原始点云

    pcl::PointCloud<PointType>::Ptr fullCloud;      // 降采样去畸变后的点云 // TODO 因为大小还是原始点云的大小，所以被降采样的部分被赋值成了0没有变过
    pcl::PointCloud<PointType>::Ptr extractedCloud; // 把有range值的fullcloud 赋值给 extractedCloud  每一行的开始和结束的索引，一维下标（每一行索引在上一行的基础上）

    int deskewFlag;   //  检测点云是否有时间字段，判断是否可用去畸变flag，初始化0，可用1，不可用-1
    cv::Mat rangeMat; // 只是一个中间变量，其实也没什么用

    // imu预积分里程计在两个scan间的增量
    bool odomDeskewFlag; // 是否成功获得增量
    double *odomTime = new double[*queueLength];
    double *odomIncreX = new double[*queueLength];
    double *odomIncreY = new double[*queueLength];
    double *odomIncreZ = new double[*queueLength];

    lio_sam::cloud_info cloudInfo; //这里面有imu是否可用，odo是否可用，lidar的位姿，scan中各点对应线号，各点与光心距离的信息
    double timeScanCur;            //当前激光帧时间
    double timeScanNext;           // 下一激光帧时间
    std_msgs::Header cloudHeader;

public:
    ImageProjection() : deskewFlag(0)
    {

        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom = nh.subscribe<nav_msgs::Odometry>(odomTopic + "_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/deskew/cloud_deskewed", 1);
        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1);

        allocateMemory();
        // resetParameters();

        // pcl控制台输出error信息
        //  https://blog.csdn.net/weixin_38258767/article/details/104243602
        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    // 预先分配内存
    void allocateMemory()
    {
        // 给所有指针分配内存空间
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        // 给 vector预分配空间
        fullCloud->points.resize(N_SCAN * Horizon_SCAN);

        // 赋值初始化为0
        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);

        cloudInfo.pointColInd.assign(N_SCAN * Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN * Horizon_SCAN, 0);

        resetParameters();
    }

    // 所有参数置位，准备下一个scan
    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection 全部默认初值为最大浮点数
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        OdomPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < *queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
            odomTime[i] = 0;
            odomIncreX[i] = 0;
            odomIncreY[i] = 0;
            odomIncreZ[i] = 0;
        }
    }

    ~ImageProjection() { delete queueLength; }

    // 把imu数据放入队列
    void imuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg)
    {
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);

        // debug IMU data
        // 这里输出的应该是雷达坐标的
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x <<
        //       ", y: " << thisImu.linear_acceleration.y <<
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x <<
        //       ", y: " << thisImu.angular_velocity.y <<
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    }

    //把imu预积分增量里程计放入队列
    void odometryHandler(const nav_msgs::Odometry::ConstPtr &odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;

        projectPointCloud();

        cloudExtraction();

        publishClouds();

        resetParameters();
    }

    void myPassThrough(const pcl::PointCloud<PointXYZIRT> &cloud_in,
                       pcl::PointCloud<PointXYZIRT> &cloud_out,
                       float limit)
    {
        // If the clouds are not the same, prepare the output
        if (&cloud_in != &cloud_out)
        {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }
        size_t j = 0;

        // #pragma omp parallel for num_threads(numberOfCores)
        for (size_t i = 0; i < cloud_in.points.size(); ++i)
        {
            if ((cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y) > limit)
                continue;
            cloud_out.points[j] = cloud_in.points[i];
            j++;
        }
        if (j != cloud_in.points.size())
        {
            // Resize to the correct size
            cloud_out.points.resize(j);
        }

        // 只有这两行不一样
        cloud_out.height = cloud_in.height;
        cloud_out.width = static_cast<uint32_t>(j) / cloud_in.height;
    }

    // void myremoveNaNFromPointCloud (const pcl::PointCloud<PointXYZIRT> &cloud_in,
    //                           pcl::PointCloud<PointXYZIRT> &cloud_out,
    //                           std::vector<int> &index)
    // {
    //     // If the clouds are not the same, prepare the output
    //     if (&cloud_in != &cloud_out)
    //     {
    //         cloud_out.header = cloud_in.header;
    //         cloud_out.points.resize (cloud_in.points.size ());
    //     }
    //     // Reserve enough space for the indices
    //     index.resize (cloud_in.points.size ());
    //     size_t j = 0;

    //     // If the data is dense, we don't need to check for NaN
    //     if (cloud_in.is_dense)
    //     {
    //         // Simply copy the data
    //         cloud_out = cloud_in;
    //         for (j = 0; j < cloud_out.points.size (); ++j)
    //         index[j] = static_cast<int>(j);
    //     }
    //     else
    //     {
    //         for (size_t i = 0; i < cloud_in.points.size (); ++i)
    //         {
    //             if (!pcl_isfinite (cloud_in.points[i].x) ||
    //                 !pcl_isfinite (cloud_in.points[i].y) ||
    //                 !pcl_isfinite (cloud_in.points[i].z))
    //                 continue;
    //                 cloud_out.points[j] = cloud_in.points[i];
    //                 cloud_out.points[j].timestamp = cloud_in.points[i].timestamp- cloud_in.points[0].timestamp;
    //                 index[j] = static_cast<int>(i);
    //                 j++;
    //         }
    //         if (j != cloud_in.points.size ())
    //         {
    //         // Resize to the correct size
    //         cloud_out.points.resize (j);
    //         index.resize (j);
    //         }

    //         // 只有这两行不一样
    //         cloud_out.height = cloud_in.height;
    //         cloud_out.width  = static_cast<uint32_t>(j)/cloud_in.height;

    //         // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
    //         cloud_out.is_dense = true;
    //     }
    // }
    //检测点云数量是否够2个，获得当前时间和下一帧点云时间
    //检查点云是否稠密、线数通道和时间通道是否存在
    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        // ROS_INFO("receive lidar");
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg); // 点云数据放入队列

        // 点云数量要大于2
        if (cloudQueue.size() <= 2)
            return false;
        else
        {
            currentCloudMsg = cloudQueue.front();
            cloudQueue.pop_front();

            cloudHeader = currentCloudMsg.header;
            timeScanCur = cloudHeader.stamp.toSec();                //队列中最早点云时间，作为当前时间
            timeScanNext = cloudQueue.front().header.stamp.toSec(); //队列中第二早点云时间,作为下一帧点云时间
        }

        // convert cloud
        pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);
        cout << "topic：" << std::to_string(currentCloudMsg.header.stamp.toSec()) << endl;
        cout << "第一个" << std::to_string(laserCloudIn->front().timestamp) << ","
             << "最后一个" << std::to_string(laserCloudIn->back().timestamp) << endl;
        cout << "intensity" << laserCloudIn->front().intensity << endl;

        // distance*distance
        // myPassThrough(*laserCloudIn,*laserCloudIn,distance_limit);

        if (mlidar_type == Velodyne)
        {
            // check dense flag
            // 检测点云是否稠密
            // if (laserCloudIn->is_dense == false)
            // {
            //     // std::cout<< "laserCloudIn->is_dense"<<laserCloudIn->is_dense<<std::endl;
            //     // ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            //     // ros::shutdown();
            //     std::vector<int> indices;
            //     myremoveNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
            // }

            // check ring channel
            // 检测是否有ring字段
            static int ringFlag = 0;
            if (ringFlag == 0)
            {
                ringFlag = -1;
                for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
                {
                    if (currentCloudMsg.fields[i].name == "ring")
                    {
                        ringFlag = 1;
                        break;
                    }
                }
                if (ringFlag == -1)
                {
                    ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                    ros::shutdown();
                }
            }

            // check point time
            // 检测点云是否有时间字段，判断是否可用去畸变
            if (deskewFlag == 0)
            {
                deskewFlag = -1;
                for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
                {
                    if (currentCloudMsg.fields[i].name == timeField)
                    {
                        deskewFlag = 1;
                        break;
                    }
                }
                if (deskewFlag == -1)
                    ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
            }
        }
        // rslidar雷达
        else if (mlidar_type == rslidar)
        {
            // check dense flag
            // 检测点云是否稠密
            // if (laserCloudIn->is_dense == false)
            // {
            //     // std::cout<< "laserCloudIn->is_dense"<<laserCloudIn->is_dense<<std::endl;
            //     // ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            //     // ros::shutdown();
            //     std::vector<int> indices;
            //     myremoveNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
            // }

            // check ring channel
            // 检测是否有ring字段
            static int ringFlag = 0;
            if (ringFlag == 0)
            {
                ringFlag = -1;
                for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
                {
                    if (currentCloudMsg.fields[i].name == "ring")
                    {
                        ringFlag = 1;
                        break;
                    }
                }
                if (ringFlag == -1)
                {
                    ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                    ros::shutdown();
                }
            }

            // check point time
            // 检测点云是否有时间字段，判断是否可用去畸变
            if (deskewFlag == 0)
            {
                deskewFlag = -1;
                for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
                {
                    if (currentCloudMsg.fields[i].name == timeField)
                    {
                        deskewFlag = 1;
                        break;
                    }
                }
                if (deskewFlag == -1)
                    ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
            }

            // #pragma omp parallel for num_threads(numberOfCores)
            // for (int h = 0; h < laserCloudIn->height - 1; ++h) {
            //     for (int w = 0; w < laserCloudIn->width - 1; ++w) {
            //         laserCloudIn->at(w, h).timestamp = currentCloudMsg.header.stamp.toSec();
            //     }
            // }
        }
        else
        {
            ROS_ERROR("Undefined lidar type");
            exit(-1);
        }

        return true;
    }

    //检查IMU队列是空，imu序列的第一个时间戳大于当前帧雷达时间戳，IMU最后一个时间戳小于下一帧雷达时间戳，则false
    // 即imu队列的时间长度大于两激光帧长度
    // 给cloud_info赋值以及获得当前scan到下帧scan的imu积分增量xyzRPY
    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanNext)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }

        imuDeskewInfo();

        odomDeskewInfo();

        return true;
    }

    // 将imu和当前scan对齐 Imu_frontTime =[timeScanCur-0.01, timeScanCur]
    // 1. 把imu与当前帧最接近的RPY赋值给cloud_info
    // 2. 计算[timeScanCur-0.01, timeScanNext+0.01] 的imu角速度积分，并判定imu数据是否可用（数量够>1）
    void imuDeskewInfo()
    {
        cloudInfo.imuAvailable = false;

        //剔除imu数据直到imu的时间戳到当前scan时间戳前0.01s以内
        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;
        double imuRotX_0, imuRotY_0, imuRotZ_0;
        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            // get roll, pitch, and yaw estimation for this scan
            // [timeScanCur-0.01, timeScanCur]中最接近的imu值
            if (currentImuTime <= timeScanCur)
                // sensor_msgs::Imu -> cloud_info::roll pitch yaw
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

            if (currentImuTime > timeScanNext + 0.01)
                break;

            // 积分的第一个值当然是0啦
            if (imuPointerCur == 0)
            {
                imuRPY2rosRPY(&thisImuMsg, &imuRotX_0, &imuRotY_0, &imuRotZ_0);
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur; // 其实这个imuPointerCur==i
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            // 旋转积分增量
            // TODO 不是有预积分的值吗，为什么还算这个
            // TODO 为什么不用真值呢？而且这样算连bias都没去掉
            double timeDiff = currentImuTime - imuTime[imuPointerCur - 1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur - 1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur - 1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur - 1] + angular_z * timeDiff;

            // 有效的
            // imuRPY2rosRPY(&thisImuMsg, &angular_x, &angular_y, &angular_z);
            // imuRotX[imuPointerCur] = angular_x - imuRotX_0;
            // imuRotY[imuPointerCur] = angular_y - imuRotY_0;
            // imuRotZ[imuPointerCur] = angular_z - imuRotZ_0 ;

            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0) // <=0说明imuQueue为空或只有1个
            return;

        cloudInfo.imuAvailable = true;
    }

    //将imu预积分里程计和当前scan对齐 startOdomMsg >=timeScanCur
    // 1. imu预积分里程计的值作为cloud_info的initialGuess值
    // 2. 和imu一样，odomQueue的时间要大于两激光帧长度，endOdomMsg>=timeScanNext
    //  则得到两个scan间的imu预积分里程xyz预积分的增量
    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;

        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        // 获取和当前scan时刻的imu预积分里程计时间
        nav_msgs::Odometry startOdomMsg;

        // startOdomMsg的时间 >=timeScanCur
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        // 这里的tf转换只是为了获得RPY
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw = yaw;

        cloudInfo.odomAvailable = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;
        OdomPointerCur = 0;

        double odomIncreX_0, odomIncreY_0, odomIncreZ_0;
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            nav_msgs::Odometry thisOdomMsg = odomQueue[i];
            double currentOdomTime = thisOdomMsg.header.stamp.toSec();

            if (currentOdomTime > timeScanNext + 0.01)
                break;

            // 积分的第一个值当然是0啦
            if (OdomPointerCur == 0)
            {
                odomIncreX_0 = thisOdomMsg.pose.pose.position.x;
                odomIncreY_0 = thisOdomMsg.pose.pose.position.y;
                odomIncreZ_0 = thisOdomMsg.pose.pose.position.z;
                odomIncreX[0] = 0;
                odomIncreY[0] = 0;
                odomIncreZ[0] = 0;
                odomTime[0] = currentOdomTime;
                ++OdomPointerCur;
                continue;
            }

            odomIncreX[OdomPointerCur] = thisOdomMsg.pose.pose.position.x - odomIncreX_0;
            odomIncreY[OdomPointerCur] = thisOdomMsg.pose.pose.position.y - odomIncreY_0;
            odomIncreZ[OdomPointerCur] = thisOdomMsg.pose.pose.position.z - odomIncreZ_0;
            odomTime[OdomPointerCur] = currentOdomTime;
            ++OdomPointerCur;
        }

        --OdomPointerCur;

        if (OdomPointerCur <= 0) // <=0说明imuQueue为空或只有1个
            return;
        // if (odomQueue.back().header.stamp.toSec() < timeScanNext)
        //     return;

        // nav_msgs::Odometry endOdomMsg;

        // // endOdomMsg的时间 >=timeScanNext
        // for (int i = 0; i < (int)odomQueue.size(); ++i)
        // {
        //     endOdomMsg = odomQueue[i];

        //     if (ROS_TIME(&endOdomMsg) < timeScanNext)
        //         continue;
        //     else
        //         break;
        // }

        // // 里程计开始和结束的方差不相等则跳过
        // // TODO 什么时候会不相等？？
        // if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
        //     return;

        // // TODO这里的预积分里程计是为了后面点云去畸变，那么这里其实可以像旋转一样更精细一点
        // Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        // tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        // Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        // Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        // float rollIncre, pitchIncre, yawIncre;
        // pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    //根据点云中某点的时间戳赋予其对应插值得到旋转量
    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0;
        *rotYCur = 0;
        *rotZCur = 0;

        int imuPointerFront = 0;
        // 一直++imuPointerFront到imu时间比pointTime略大
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        // 都++完了也没到while循环里的break要求，直接用了最后的imu数据
        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        }
        else
        { // 有满足条件的imu数据，前后两点插值
            // cout<<"orientation对齐成功"<<endl;
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    // 根据点云中某点的时间戳赋予其对应插值得到imu预积分里程计的位移增量
    void findPosition(double pointTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0;
        *posYCur = 0;
        *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        int OdomPointerFront = 0;

        while (OdomPointerFront < OdomPointerCur)
        {
            if (pointTime < odomTime[OdomPointerFront])
                break;
            ++OdomPointerFront;
        }

        // 都++完了也没到while循环里的break要求，直接用了最后的imu数据
        if (pointTime > odomTime[OdomPointerFront] || OdomPointerFront == 0)
        {
            *posXCur = odomIncreX[OdomPointerFront];
            *posYCur = odomIncreY[OdomPointerFront];
            *posZCur = odomIncreZ[OdomPointerFront];
        }
        else
        { // 有满足条件的数据，前后两点插值

            // cout<<"Position对齐成功"<<endl;
            int OdomPointerBack = OdomPointerFront - 1;
            double ratioFront = (pointTime - odomTime[OdomPointerBack]) / (odomTime[OdomPointerFront] - odomTime[OdomPointerBack]);
            double ratioBack = (odomTime[OdomPointerFront] - pointTime) / (odomTime[OdomPointerFront] - odomTime[OdomPointerBack]);
            *posXCur = odomIncreX[OdomPointerFront] * ratioFront + odomIncreX[OdomPointerBack] * ratioBack;
            *posYCur = odomIncreY[OdomPointerFront] * ratioFront + odomIncreY[OdomPointerBack] * ratioBack;
            *posZCur = odomIncreZ[OdomPointerFront] * ratioFront + odomIncreZ[OdomPointerBack] * ratioBack;
        }

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanNext - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    // 点云去畸变，用imu预积分的xyzRPY校正点云都到起始点坐标系下
    //如果没办法去畸变(没有时间戳，没有imu数据)就返回原点 //TODO没有imu数据也可以去畸变呀,实际工程可以增加imu失效的情况
    PointType deskewPoint(PointType *point, double relTime) // 点的间隔时间
    {
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        double pointTime = timeScanCur + relTime; // 点时间=激光scan开始时间+点的间隔时间

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(pointTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        // 全部转到起始点坐标系下的旋转矩阵
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0, 0) * point->x + transBt(0, 1) * point->y + transBt(0, 2) * point->z + transBt(0, 3);
        newPoint.y = transBt(1, 0) * point->x + transBt(1, 1) * point->y + transBt(1, 2) * point->z + transBt(1, 3);
        newPoint.z = transBt(2, 0) * point->x + transBt(2, 1) * point->y + transBt(2, 2) * point->z + transBt(2, 3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    // 和 lego-loam类似，增加了一步点云去畸变
    // 投影到rang image图像中,逐点计算点云深度，
    int rowIdn_max = 0, rowIdn_min = 999;
    void projectPointCloud()
    {
        int rowIdn;

        float verticalAngle, horizonAngle;
        static float ang_res_x = 360.0 / float(Horizon_SCAN);
        static float ang_res_y = Vertical_angle / float(N_SCAN - 1);
        // range image projection
        // for (int i = 0; i < cloudSize; ++i)

        int cloudSize = laserCloudIn->points.size();
        for (int i = 0; i < cloudSize; ++i)
        {

            if (!pcl_isfinite(laserCloudIn->points[i].x) ||
                !pcl_isfinite(laserCloudIn->points[i].y) ||
                !pcl_isfinite(laserCloudIn->points[i].z))
                continue;
            if (laserCloudIn->points[i].z < -3.)
                continue;
            // TODO 针对现在的车设置的
            if (abs(laserCloudIn->points[i].x) < 1.1 && abs(laserCloudIn->points[i].y) < 2.5)
                continue;
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            // 算在哪个水平直线上
            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            // 如果有降采样，则每隔downsampleRate个取一个
            if (rowIdn % downsampleRate != 0)
                continue;

            // 该点的水平角度
            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            // 水平分辨率
            // static float ang_res_x = 360.0/float(Horizon_SCAN);
            // 算在哪个竖直线id上
            int columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            // 获得到原点的距离
            float range = pointDistance(thisPoint);

            if (range > max_range)
                continue;

            // TODO这个情况可能发生吗？？
            // 已经赋过值则跳过
            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            //
            if (mlidar_type == rslidar)
                thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].timestamp - timeScanCur); // rslidar
            else if (mlidar_type == Velodyne)
                thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].timestamp); // Velodyne
            // thisPoint = deskewPoint(&thisPoint, (float)laserCloudIn->points[i].t / 1000000000.0); // Ouster

            // TODO 算了两次，为什么不直接赋值range
            // TODO 前面算的没有去畸变，那为什么不去畸变再算呢？？
            rangeMat.at<float>(rowIdn, columnIdn) = pointDistance(thisPoint);

            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
        // 去完畸变清理一下
        for (int i = 0; i < *queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
            odomTime[i] = 0;
            odomIncreX[i] = 0;
            odomIncreY[i] = 0;
            odomIncreZ[i] = 0;
        }
    }

    //把有range值的fullcloud 赋值给 extractedCloud
    //记录startRingIndex、endRingIndex、pointColInd、pointRange
    void cloudExtraction()
    {
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i) // 16
        {
            cloudInfo.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j) // 1800
            {
                if (rangeMat.at<float>(i, j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.pointColInd[count] = j;
                    // cloudInfo.pointRowInd[count] = i;
                    // save range info
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i, j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count - 1 - 5;
        }
    }

    //发布去畸变和无效点后的点云和 cloudInfo
    // 此时的cloudInfo包含除角点和面点点云以外的所有信息
    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo.publish(cloudInfo);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_sam");

    ImageProjection IP;

    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}