// TODO 本code创新点：用关键帧队列存储点云，避免了在整个地图里搜索点云
#include "Scancontext.h"
#include "utility.h"
#include "dynamic_map.h"
#include "lio_sam/cloud_info.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <pcl/filters/passthrough.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>
using namespace gtsam;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

/*
 * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
 */
#if 0
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))
#endif
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY; // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, timestamp, timestamp))

typedef PointXYZIRPYT PointTypePose;

class mapOptimization : public ParamServer
{

public:
    int map_count = 0;
    pcl::PointCloud<PointType>::Ptr globalMap;
    bool initial = false;                                      // 初始化位姿
    bool load_map = false;                                     //
    pcl::Registration<PointType, PointType>::Ptr registration; //匹配方法

    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate; // 初始位姿估计

    ISAM2 *isam; // 优化器推理算法
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance; // 优化后的上一关键帧位姿协方差

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher pubLaserOdometryIncremental;

    ros::Publisher pubPath;

    ros::Publisher pubCloudRegisteredRaw;

    ros::Subscriber subCloud; // 输入点云
    ros::Subscriber subGPS;   // 输入gps
    ros::Subscriber subUKF;

    std::deque<nav_msgs::Odometry> gpsQueue; // GPS队列
    lio_sam::cloud_info cloudInfo;           // 用来存储topic接收的点云

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;     // gtsam优化后的地图关键帧位置(x，y，z)
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D; //优化后的地图关键帧位置（x，y，z，R, P,Y，time）

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;   // topic接收到的角点点云,当前点云 corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfCur;      // topic接收到的平面点云surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;     // topic接收到的平面点云surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfCurDS;    // downsampled surf featuer set from odoOptimization
    pcl::PointCloud<PointType>::Ptr cloudScanForInitialize;

    pcl::PointCloud<PointType>::Ptr laserCloudOri; // 经过筛选的可以用于匹配的点
    pcl::PointCloud<PointType>::Ptr coeffSel;      // 优化方向的向量的系数

    std::vector<PointType> laserCloudOriCornerVec; // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;   // 从地图中提取的除当前帧外的当前帧的周围点云
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;     // 从地图中提取的除当前帧外的当前帧的周围点云
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS; // 上面的降采样
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;   // 上面的降采样

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeLastScan;

    pcl::VoxelGrid<PointType> downSizeFilterMap;
    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;

    pcl::PassThrough<PointType> pass;

    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;
    double timeLaserInfoLast = -1;

    float transformTobeMapped[6]; //当前的里程计 （初始数据来自imu积分里程计，然后经过scan2map优化），  RPYxyz初始化为0,0,0,0,0,0
    float last_loadMap[6];
    std::mutex mtx;
    std::mutex Mapmtx;

    bool isDegenerate = false;
    Eigen::Matrix<float, 6, 6> matP;

    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfCurDSNum = 0;
    nav_msgs::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;       // transformTobeMapped的矩阵形式
    Eigen::Affine3f incrementalOdometryAffineFront; // save current transformation before any processing 相当于slam里程计值
    Eigen::Affine3f incrementalOdometryAffineBack;  //  经过scan2map优化后的值，又经过了imu差值后的值
    Eigen::Matrix4f D_T;                            // Ti-1,i

    double Corner_fitness_score, Surf_fitness_score;
    int Corner_num = 0, Surf_num = 0;
    bool gps_initailized;
    bool map_initailized;
    bool pose_initailized;
    int initial_count = 0;
    bool lose_flag;
    bool ukf_initialized = false;
    enum InitializedFlag
    {
        NonInitialized,
        Initializing,
        Initialized
    };
    InitializedFlag initializedFlag;
    pclomp::NormalDistributionsTransform<PointType, PointType> relocalization_ndt;
    pclomp::GeneralizedIterativeClosestPoint<PointType, PointType> relocalization_gicp;
    mapOptimization() : gps_initailized(false), map_initailized(false), pose_initailized(false), lose_flag(false)
    {

        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_global", 1);
        pubLaserOdometryGlobal = nh.advertise<nav_msgs::Odometry>("lio_sam/mapping/odometry", 1);                  // 全局里程计，有优化
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 1); // 发布增量里程计，不受闭环优化等的影响，只用来算增量，是为了防止在算增量过程中发生优化，导致增量出错
        pubPath = nh.advertise<nav_msgs::Path>("lio_sam/mapping/path", 1);                                         // 发布路径

        subCloud = nh.subscribe<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        // subGPS = nh.subscribe<nav_msgs::Odometry>(gpsTopic, 200, &mapOptimization::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        subUKF = nh.subscribe<nav_msgs::Odometry>("ukf/odom", 1, &mapOptimization::UKFHandler, this, ros::TransportHints().tcpNoDelay());

        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered_raw", 1);

        // downSizeFilterMap.setLeafSize(1.0, 1.0, 1.0);
        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize * 1.5, mappingCornerLeafSize * 1.5, mappingCornerLeafSize * 1.5);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize * 1.5, mappingSurfLeafSize * 1.5, mappingSurfLeafSize * 1.5);

        relocalization_ndt.setTransformationEpsilon(0.01);
        relocalization_ndt.setResolution(ndt_resolution);
        relocalization_ndt.setTransformationEpsilon(0.01);
        relocalization_ndt.setResolution(1.0);
        // 设置搜索方法
        if (ndt_neighbor_search_method == "DIRECT1")
        {

            relocalization_ndt.setNeighborhoodSearchMethod(pclomp::DIRECT1);
        }
        else if (ndt_neighbor_search_method == "DIRECT7")
        {

            relocalization_ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
        }
        else
        {
            if (ndt_neighbor_search_method == "KDTREE")
            {
                ROS_INFO("search_method KDTREE is selected");
            }
            else
            {
                ROS_INFO("invalid search method was given");
                ROS_INFO("default method is selected (KDTREE)");
            }
            relocalization_ndt.setNeighborhoodSearchMethod(pclomp::KDTREE);
        }

        relocalization_gicp.setMaxCorrespondenceDistance(100);
        relocalization_gicp.setMaximumIterations(100);
        relocalization_gicp.setTransformationEpsilon(1e-6);
        relocalization_gicp.setEuclideanFitnessEpsilon(1e-6);
        relocalization_gicp.setRANSACIterations(0);

        if (mintialMethod == human)
            gps_initailized = true;
        allocateMemory();
    }

    // 预先分配内存
    void allocateMemory()
    {

        initializedFlag = NonInitialized;

        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        cloudScanForInitialize.reset(new pcl::PointCloud<PointType>());

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        laserCloudSurfCur.reset(new pcl::PointCloud<PointType>());    // surf feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
        laserCloudSurfCurDS.reset(new pcl::PointCloud<PointType>());    // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMap->header.frame_id = "slam_map";
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap->header.frame_id = "slam_map";
        globalMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());
        //动态加载地图
        all_Corner_areas = read_arealist(globalCornerMap_pcd);
        all_Surf_areas = read_arealist(globalSurfMap_pcd);
        std::cout << "角点地图张： " << all_Corner_areas.size() << std::endl;
        std::cout << "面点地图张： " << all_Surf_areas.size() << std::endl;

        if (margin < 0) // margin<0，说明不更新，那么加载全部pcd文件
        {
            for (const Area &area : all_Corner_areas)
            {
                Corner_pcd_file_paths.push_back(globalCornerMap_dirctory + area.path);
            }
            for (const Area &area : all_Surf_areas)
            {
                Surf_pcd_file_paths.push_back(globalSurfMap_dirctory + area.path);
            }
            laserCloudCornerFromMap = create_pcd(Corner_pcd_file_paths);
            laserCloudSurfFromMap = create_pcd(Surf_pcd_file_paths);

            load_map = true;
        }

        // 静态加载地图
        // pcl::io::loadPCDFile(globalCornerMap_pcd, *laserCloudCornerFromMap);

        // pcl::io::loadPCDFile(globalSurfMap_pcd, *laserCloudSurfFromMap);

        // *globalMap =*laserCloudCornerFromMap+ *laserCloudSurfFromMap;
        // publishCloud(&pubLaserCloudSurround, globalMap, ros::Time::now(), "map");

        // laserCloudCornerFromMapDS = laserCloudCornerFromMap;
        // laserCloudSurfFromMapDS = laserCloudSurfFromMap;
        // downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        // downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        // downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        // downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeLastScan.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i)
        {
            transformTobeMapped[i] = 0;
            last_loadMap[i] = -999999;
        }

        matP.setZero();

        // 加载地图
    }

    void myPassThrough(const pcl::PointCloud<PointType> &cloud_in,
                       pcl::PointCloud<PointType> &cloud_out,
                       float limit_xl, float limit_xr,
                       float limit_yl, float limit_yr)
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
            if (sqrt((cloud_in.points[i].x - transformTobeMapped[3]) * (cloud_in.points[i].x - transformTobeMapped[3]) + (cloud_in.points[i].y - transformTobeMapped[4]) * (cloud_in.points[i].y - transformTobeMapped[4])) > max_range || cloud_in.points[i].z > 10)
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

    void dynamic_load_map(const float pose[])
    {
        if (margin >= 0)
        {

            // laserCloudCornerFromMapDS = laserCloudCornerFromMap;
            // laserCloudSurfFromMapDS = laserCloudSurfFromMap;

            // downSizeFilterMap.setInputCloud(laserCloudCornerFromMap);
            // downSizeFilterMap.filter(*laserCloudCornerFromMapDS);
            // downSizeFilterMap.setInputCloud(laserCloudSurfFromMap);
            // downSizeFilterMap.filter(*laserCloudSurfFromMapDS);

            // pass.setInputCloud (laserCloudCornerFromMap);
            // pass.setFilterFieldName ("x");// 滤波字段设置为z轴方向
            // pass.setFilterLimits (pose[3]-max_range-10, pose[3]+max_range+10);
            // pass.filter (*laserCloudCornerFromMapDS);
            // pass.setInputCloud (laserCloudSurfFromMap);
            // pass.filter (*laserCloudSurfFromMapDS);

            // pass.setInputCloud (laserCloudCornerFromMap);
            // pass.setFilterFieldName ("y");// 滤波字段设置为z轴方向
            // pass.setFilterLimits (pose[4]-max_range-10, pose[4]+max_range+10);
            // pass.filter (*laserCloudCornerFromMapDS);

            // pass.setInputCloud (laserCloudSurfFromMap);
            // pass.filter (*laserCloudSurfFromMapDS);
            std::cout << "当前坐标" << pose[3] << "," << pose[4] << std::endl;
            myPassThrough(*laserCloudCornerFromMap, *laserCloudCornerFromMapDS,
                          pose[3] - max_range - 5, pose[3] + max_range + 5, pose[4] - max_range - 5, pose[4] + max_range + 5);
            myPassThrough(*laserCloudSurfFromMap, *laserCloudSurfFromMapDS,
                          pose[3] - max_range - 5, pose[3] + max_range + 5, pose[4] - max_range - 5, pose[4] + max_range + 5);
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            *globalMap = *laserCloudCornerFromMapDS + *laserCloudSurfFromMap;

            // if(Matching_method=="ndt")
            //     registration->setInputTarget(globalMap);
        }
    }
    void dynamic_load_map_run()
    {
        ros::Rate rate(updateMapFrequency);
        while (ros::ok())
        {
            if (!gps_initailized)
                continue;
            if (!map_initailized)
            {
                // continue;
                transformTobeMapped[3] = initialPose[0];
                transformTobeMapped[4] = initialPose[1];
                transformTobeMapped[5] = initialPose[2];
                cout << "initial pose is" << initialPose[0] << "," << initialPose[1] << "," << initialPose[2] << endl;
            }

            float distance_x = transformTobeMapped[3] - last_loadMap[3];
            float distance_y = transformTobeMapped[4] - last_loadMap[4];
            float distance_z = transformTobeMapped[5] - last_loadMap[5];
            float load_distance = sqrt(distance_x * distance_x + distance_y * distance_y + distance_z * distance_z);
            // std::cout<<"load_distance"<<load_distance<<std::endl;

            if (load_distance > area_size * 0.4)
            {
                std::lock_guard<std::mutex> lock(Mapmtx);
                std::cout << "加载地图" << std::endl;
                laserCloudCornerFromMap = create_pcd(transformTobeMapped[3], transformTobeMapped[4], all_Corner_areas, globalCornerMap_dirctory, margin);
                laserCloudSurfFromMap = create_pcd(transformTobeMapped[3], transformTobeMapped[4], all_Surf_areas, globalSurfMap_dirctory, margin);
                dynamic_load_map(transformTobeMapped);
                publishCloud(&pubLaserCloudSurround, globalMap, ros::Time::now(), "slam_map");
                map_initailized = true;
                for (int i = 3; i < 6; ++i)
                {
                    last_loadMap[i] = transformTobeMapped[i];
                }
                load_map = true;
                map_count = 0;
            }
            else
            {
                std::lock_guard<std::mutex> lock(Mapmtx);
                map_count++;
                map_count %= 10;
                if (map_count == 0)
                {
                    dynamic_load_map(transformTobeMapped);
                }
            }
            // else
            rate.sleep();
        }
    }

    void ndt_registration()
    {
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        *cloud = *laserCloudCornerLastDS + *laserCloudSurfCurDS;
        Eigen::Affine3f init_guess = trans2Affine3f(transformTobeMapped);

        // std::cout<<"初始位姿 ="<<pos()<<std::endl;
        pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>());
        registration->setInputSource(cloud);
        registration->align(*aligned, init_guess.matrix());
        std::cout << "hasConverged =" << registration->hasConverged() << std::endl;
        std::cout << "getFitnessScore =" << registration->getFitnessScore() << std::endl;
        // if(registration->getFitnessScore()<0.1){
        Eigen::Matrix4f trans = registration->getFinalTransformation();
        Eigen::Vector3f p = trans.block<3, 1>(0, 3);
        Eigen::Matrix3f R = trans.block<3, 3>(0, 0);
        Eigen::Vector3f eulerAngle = R.eulerAngles(2, 1, 0);
        // float x, y, z, roll, pitch, yaw;
        // pcl::getTranslationAndEulerAngles(trans, x, y, z, roll, pitch, yaw);
        transformTobeMapped[0] = eulerAngle[2];
        transformTobeMapped[1] = eulerAngle[1];
        transformTobeMapped[2] = eulerAngle[0];
        transformTobeMapped[3] = p[0];
        transformTobeMapped[4] = p[1];
        transformTobeMapped[5] = p[2];
        transformUpdate();
    }

    double my_getFitnessScore(pcl::PointCloud<PointType>::Ptr input_cloud, double max_range)
    {

        updatePointAssociateToMap();

        double fitness_score = 0.0;

        // For each point in the source dataset
        int nr = 0;
        kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
#pragma omp parallel for num_threads(numberOfCores)
        for (size_t i = 0; i < input_cloud->size(); ++i)
        {
            PointType pointOri, pointSel;
            std::vector<int> nn_indices;
            std::vector<float> nn_dists;
            pointOri = input_cloud->points[i];         //当前点在scan坐标系下的坐标
            pointAssociateToMap(&pointOri, &pointSel); // 当前点在map坐标系下的坐标
            // Find its nearest neighbor in the target
            kdtreeCornerFromMap->nearestKSearch(pointSel, 1, nn_indices, nn_dists);

            // Deal with occlusions (incomplete targets)
            if (nn_dists[0] <= max_range)
            {
                // Add to the fitness score
                fitness_score += nn_dists[0];
                nr++;
            }
        }

        if (nr > input_cloud->points.size() * 0.3)
            return (fitness_score / nr);
        else
            return (std::numeric_limits<double>::max());
    }

    void ICPLocalizeInitialize()
    {

        if (cloudScanForInitialize->points.size() == 0)
            return;

        std::cout << "the size of incoming lasercloud: " << cloudScanForInitialize->points.size() << std::endl;

        std::cout << "the pose before initializing is: x" << transformTobeMapped[3] << " y" << transformTobeMapped[4]
                  << " z" << transformTobeMapped[5] << std::endl;

        pclomp::NormalDistributionsTransform<PointType, PointType> ndt;
        pclomp::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
        ndt.setTransformationEpsilon(0.01);
        ndt.setResolution(1.0);
        // 设置搜索方法
        if (ndt_neighbor_search_method == "DIRECT1")
        {

            ndt.setNeighborhoodSearchMethod(pclomp::DIRECT1);
        }
        else if (ndt_neighbor_search_method == "DIRECT7")
        {

            ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
        }
        else
        {
            if (ndt_neighbor_search_method == "KDTREE")
            {
                ROS_INFO("search_method KDTREE is selected");
            }
            else
            {
                ROS_INFO("invalid search method was given");
                ROS_INFO("default method is selected (KDTREE)");
            }
            ndt.setNeighborhoodSearchMethod(pclomp::KDTREE);
        }

        gicp.setMaxCorrespondenceDistance(5);
        gicp.setMaximumIterations(100);
        gicp.setTransformationEpsilon(1e-6);
        gicp.setEuclideanFitnessEpsilon(1e-6);
        gicp.setRANSACIterations(0);

        ndt.setInputSource(cloudScanForInitialize);
        ndt.setInputTarget(globalMap);
        pcl::PointCloud<PointType>::Ptr unused_result_0(new pcl::PointCloud<PointType>());

        Eigen::Affine3f init_guess = trans2Affine3f(transformTobeMapped);
        ndt.align(*unused_result_0, init_guess.matrix());

        // use the outcome of ndt as the initial guess for ICP
        gicp.setInputSource(cloudScanForInitialize);
        gicp.setInputTarget(globalMap);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        gicp.align(*unused_result, ndt.getFinalTransformation());

        std::cout << "the icp score in initializing process is: " << gicp.getFitnessScore() << std::endl;
        std::cout << "the pose after initializing process is: " << gicp.getFinalTransformation() << std::endl;

        Eigen::Affine3f T_thisPose6DInMap;
        T_thisPose6DInMap = gicp.getFinalTransformation();
        pcl::getTranslationAndEulerAngles(T_thisPose6DInMap, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

        if (gicp.hasConverged() == false || gicp.getFitnessScore() > historyKeyframeFitnessScore * 2.0)
        {
            initializedFlag = Initializing;
            cloudScanForInitialize->clear();
            std::cout << "Initializing Fail" << std::endl;
            return;
        }
        else
        {
            initializedFlag = Initialized;
            std::cout << "Initializing Succeed" << std::endl;
        }

        // cloudScanForInitialize.reset(new pcl::PointCloud<PointType>());
    }

    double max_angle = 0;
    double y_v = 0;
    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr &msgIn)
    {
        // cout << "??????"<<endl;
        if (!gps_initailized)
        {
            cout << "GPS not initailized" << endl;
            return;
        }

        if (!map_initailized)
        {
            cout << "Map not initailized" << endl;
            return;
        }

        TicToc t1(true);
        TicToc tv(true);
        // extract time stamp
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = msgIn->header.stamp.toSec();

        // extract info and feature cloud
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_corner, *laserCloudCornerLast);
        pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfCur);

        std::lock_guard<std::mutex> lock(Mapmtx);
        // static double timeLastProcessing = -1; // 上次运行这部分代码时间戳
        // 和上次建图间隔时间超过mappingProcessInterval才进行建图
        // if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
        // {
        //     timeLastProcessing = timeLaserInfoCur;
        // 更新当前的里程计初值估计
        updateInitialGuess();

        if (load_map == false)
            return;
        // 降采样当前scan
        downsampleCurrentScan();
        // t1.toc("降采样");

        // map_count++;
        // map_count%=10;
        // if(map_count==0){
        //     dynamic_load_map(transformTobeMapped);
        // }

        // if the sysytem is not initialized ffer the first scan for the system to initialize
        // the LIO system stsrt working only when the localization initializing is finished
        if (initializedFlag == NonInitialized || initializedFlag == Initializing)
        {
            if (cloudScanForInitialize->points.size() == 0)
            {
                // downsampleCurrentScan();

                *cloudScanForInitialize += *laserCloudCornerLastDS;

                laserCloudCornerLastDS->clear();
                laserCloudCornerLastDSNum = 0;
                laserCloudSurfCurDSNum = 0;
            }

            if (initializedFlag == NonInitialized || initializedFlag == Initializing)
            {
                ICPLocalizeInitialize();
                initialPose[0] = transformTobeMapped[3];
                initialPose[1] = transformTobeMapped[4];
                initialPose[2] = transformTobeMapped[5];
            }
            else if (initializedFlag == Initializing)
            {
                std::cout << "Offer A New Guess Please " << std::endl; // do nothing, wait for a new initial guess
                ros::Duration(0.05).sleep();
            }

            return;
        }

        // t1.toc("加载地图");
        // if(Matching_method=="loam"){
        scan2MapOptimization();
        // std::cout<<"匹配分数是"<<my_getFitnessScore(laserCloudCornerLastDS,0.4)<<std::endl;
        std::cout << "焦点匹配分数是" << Corner_fitness_score << std::endl;
        std::cout << "面点匹配分数是" << Surf_fitness_score << std::endl;
        // }
        // else if(Matching_method=="ndt"){
        //     ndt_registration();
        // }
        // t1.toc("youhua");
        initial_count++;
        lose_flag = false;
        // 根据运动模型判断是否丢定位
        // {
        D_T = (incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack).matrix();
        Eigen::Matrix3f Rotation = D_T.block<3, 3>(0, 0);
        Eigen::Vector3f Trans = D_T.block<3, 1>(0, 3);
        Eigen::Vector3f eulerAngle = Rotation.eulerAngles(0, 1, 2);
        float yaw_180 = abs(eulerAngle(2)) / M_PI * 180.;
        cout << "yaw_180: " << yaw_180 << endl;

        yaw_180 = (180. - abs(yaw_180)) < 40. ? 180. - abs(yaw_180) : yaw_180;
        cout << "yaw_180_after: " << yaw_180 << endl;
        double delta_time = timeLaserInfoLast == -1 ? 0.1 : timeLaserInfoCur - timeLaserInfoLast;
        cout << "delta_time: " << delta_time << endl;
        // if((Trans.norm()*10.> 12 || (Trans.norm()*10.>1 && abs(eulerAngle(2))*10.>0.25))&&initial_count>initial_count_num*3)
        // TODO 针对我们的车，x是横向 Trans(0)/delta_time =速度
        // TODO eulerAngle is in rads
        // x max = 3.71 yaw_max = 105
        if ((yaw_180 / delta_time > 100 || abs(Trans(0) / delta_time) > 3.0 || abs(Trans(1) / delta_time) > 12 || (yaw_180 / delta_time < 30 && abs(Trans(0) / delta_time) > 1.6)) && initial_count > initial_count_num * 3)
            lose_flag = true;
        max_angle = max(yaw_180 / delta_time, max_angle);
        y_v = max(y_v, abs(Trans(0) / delta_time));
        cout << "当前的速度是" << Trans / delta_time << "," << yaw_180 / delta_time << endl;
        cout << "zuidasudu " << max_angle << "," << y_v << endl;

        // }

        if (!lose_flag && initial_count > initial_count_num)
        {
            // saveKeyFramesAndFactor();
            if (initial_count > initial_count_num * 3)
                initial_count = initial_count_num * 3;
            publishOdometry();
            publishFrames();
        }
        else if (initial_count < initial_count_num + 1)
        {
            publishOdometry();
            publishFrames();
        }
        // else if(lose_flag){
        //     publishOdometry();
        //     publishFrames();
        // }

        else
        {

            // // ros::shutdown();
            cout << "定位失败重定位,当前的速度是" << Trans / delta_time << "," << abs(eulerAngle(2)) / delta_time << endl;
            // scan2scanOptimization();
            // // 还原原来的值
            // Eigen::Matrix4f tmp = incrementalOdometryAffineFront.matrix() *D_T;
            // Eigen::Matrix3f Rotation = tmp.block<3,3>(0,0);
            // Eigen::Vector3f Trans = tmp.block<3,1>(0,3);
            // Eigen::Vector3f eulerAngle=Rotation.eulerAngles(0,1,2);
            // transformTobeMapped[0] = eulerAngle(0);
            // transformTobeMapped[1] = eulerAngle(1);
            // transformTobeMapped[2] = eulerAngle(2);
            // // cout<<"initial pose is"<<initialPose[0]<<","<<initialPose[1]<<","<<initialPose[2]<<endl;
            // transformTobeMapped[3] = Trans(0);
            // transformTobeMapped[4] = Trans(1);
            // transformTobeMapped[5] = Trans(2);
            //   // 迭代结束更新相关的转移矩阵
            // transformUpdate();
            // if(Surf_fitness_score<0.25){
            //     publishOdometry();
            //     publishFrames();
            //     std::cout << "Relocalization Succeed" << std::endl;

            // }
            // else{
            //     std::cout << "Relocalization Fail" << std::endl;
            // }

            {
                std::lock_guard<std::mutex> lock(mtx);
                transformTobeMapped[0] = cloudInfo.imuRollInit;
                transformTobeMapped[1] = cloudInfo.imuPitchInit;
                transformTobeMapped[2] = cloudInfo.imuYawInit;
                // cout<<"initial pose is"<<initialPose[0]<<","<<initialPose[1]<<","<<initialPose[2]<<endl;
                transformTobeMapped[3] = initialPose[0];
                transformTobeMapped[4] = initialPose[1];
                transformTobeMapped[5] = initialPose[2];
            }

            // relocalization_ndt.setInputSource(tempCloud);
            // relocalization_ndt.setInputTarget(globalMap);
            // pcl::PointCloud<PointType>::Ptr unused_result_0(new pcl::PointCloud<PointType>());

            Eigen::Affine3f init_guess = trans2Affine3f(transformTobeMapped);
            // relocalization_ndt.align(*unused_result_0, init_guess.matrix());

            relocalization_gicp.setMaxCorrespondenceDistance(5);
            relocalization_gicp.setMaximumIterations(100);
            relocalization_gicp.setTransformationEpsilon(1e-6);
            relocalization_gicp.setEuclideanFitnessEpsilon(1e-6);
            relocalization_gicp.setRANSACIterations(0);

            // use the outcome of ndt as the initial guess for ICP
            //  relocalization_gicp.setMaximumIterations(10);
            //  relocalization_gicp.setRotationEpsilon(0.05);
            //  relocalization_gicp.setTransformationEpsilon(0.1);
            relocalization_gicp.setInputSource(laserCloudCornerLastDS);
            relocalization_gicp.setInputTarget(globalMap);
            pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
            relocalization_gicp.align(*unused_result, init_guess.matrix());

            Eigen::Affine3f T_thisPose6DInMap;
            T_thisPose6DInMap = relocalization_gicp.getFinalTransformation();
            pcl::getTranslationAndEulerAngles(T_thisPose6DInMap, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            if (relocalization_gicp.hasConverged() == false || relocalization_gicp.getFitnessScore() > historyKeyframeFitnessScore * 1.5)
            {
                // {
                // std::lock_guard<std::mutex> lock(mtx);
                // transformTobeMapped[0] = cloudInfo.imuRollInit;
                // transformTobeMapped[1] = cloudInfo.imuPitchInit;
                // transformTobeMapped[2] = cloudInfo.imuYawInit;
                // // cout<<"initial pose is"<<initialPose[0]<<","<<initialPose[1]<<","<<initialPose[2]<<endl;
                // transformTobeMapped[3] = initialPose[0];
                // transformTobeMapped[4] = initialPose[1];
                // transformTobeMapped[5] = initialPose[2];
                // }
                // publishOdometry();
                // publishFrames();
                lose_flag = true;
                std::cout << "Relocalization Fail" << std::endl;
                // TODO 正式使用一定要关掉
                ros::shutdown();
                // return;
            }
            else
            {
                transformUpdate();
                publishOdometry();
                publishFrames();
                std::cout << "Relocalization Succeed" << std::endl;
            }
        }

        // if(Corner_fitness_score<0.15){
        //     lose_flag=false;
        // }
        // else{
        //     cout<<"定位失败重定位"<<endl;
        //     lose_flag=true;
        // }
        updatePath(transformTobeMapped);
        *laserCloudSurfLast = *laserCloudSurfCur;
        timeLaserInfoLast = timeLaserInfoCur;
        t1.toc("运行用时");
        // }
    }

    void UKFHandler(const nav_msgs::Odometry::ConstPtr &ukfMsg)
    {

        if (!ukf_initialized)
            ukf_initialized = true;
        Eigen::Vector3d Pwl;
        Eigen::Vector3d Pwi(ukfMsg->pose.pose.position.x, ukfMsg->pose.pose.position.y, ukfMsg->pose.pose.position.z);
        Eigen::Quaterniond Qwi(ukfMsg->pose.pose.orientation.w, ukfMsg->pose.pose.orientation.x, ukfMsg->pose.pose.orientation.y, ukfMsg->pose.pose.orientation.z);
        Pwl = Pwi + Qwi.matrix() * Pil;
        std::lock_guard<std::mutex> lock(mtx);
        initialPose.at(0) = Pwl.x();
        initialPose.at(1) = Pwl.y();
        initialPose.at(2) = Pwl.z();
    }
    // 添加GPS里程计数据到队列
    int gps_count = 0;
    std::chrono::steady_clock::time_point now;
    std::chrono::steady_clock::time_point last;
    void gpsHandler(const nav_msgs::Odometry::ConstPtr &gpsMsg)
    {

        if (mintialMethod == gps)
        {
            if (!gps_initailized && (gpsMsg->pose.pose.position.x != 0 || gpsMsg->pose.pose.position.y != 0) && (gpsMsg->pose.covariance[0] < 3 && gpsMsg->pose.covariance[7] < 3))
            {
                std::lock_guard<std::mutex> lock(mtx);
                Eigen::Vector3d Pwl;
                Eigen::Vector3d Pwi(gpsMsg->pose.pose.position.x, gpsMsg->pose.pose.position.y, gpsMsg->pose.pose.position.z);
                Eigen::Quaterniond Qwi(gpsMsg->pose.pose.orientation.w, gpsMsg->pose.pose.orientation.x, gpsMsg->pose.pose.orientation.y, gpsMsg->pose.pose.orientation.z);
                Pwl = Pwi + Qwi.matrix() * Pil;
                cout << "GPS initailizes" << endl;
                initialPose.at(0) = Pwl.x();
                initialPose.at(1) = Pwl.y();

                gps_initailized = true;
            }
        }

        // gpsQueue.push_back(*gpsMsg);
        // cout<<"收到GPS"<<endl;
    }

    // scan坐标系下的点->地图坐标系下
    void pointAssociateToMap(PointType const *const pi, PointType *const po)
    {
        po->x = transPointAssociateToMap(0, 0) * pi->x + transPointAssociateToMap(0, 1) * pi->y + transPointAssociateToMap(0, 2) * pi->z + transPointAssociateToMap(0, 3);
        po->y = transPointAssociateToMap(1, 0) * pi->x + transPointAssociateToMap(1, 1) * pi->y + transPointAssociateToMap(1, 2) * pi->z + transPointAssociateToMap(1, 3);
        po->z = transPointAssociateToMap(2, 0) * pi->x + transPointAssociateToMap(2, 1) * pi->y + transPointAssociateToMap(2, 2) * pi->z + transPointAssociateToMap(2, 3);
        po->intensity = pi->intensity;
    }

    void pointTransForm(PointType const *const pi, PointType *const po, Eigen::Matrix4f TransForm)
    {
        po->x = TransForm(0, 0) * pi->x + TransForm(0, 1) * pi->y + TransForm(0, 2) * pi->z + TransForm(0, 3);
        po->y = TransForm(1, 0) * pi->x + TransForm(1, 1) * pi->y + TransForm(1, 2) * pi->z + TransForm(1, 3);
        po->z = TransForm(2, 0) * pi->x + TransForm(2, 1) * pi->y + TransForm(2, 2) * pi->z + TransForm(2, 3);
        po->intensity = pi->intensity;
    }

    // 对输入点云进行位姿变换
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

// https://blog.csdn.net/bigFatCat_Tom/article/details/98493040
// 使用多线程并行加速
#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            pointFrom = &cloudIn->points[i];
            cloudOut->points[i].x = transCur(0, 0) * pointFrom->x + transCur(0, 1) * pointFrom->y + transCur(0, 2) * pointFrom->z + transCur(0, 3);
            cloudOut->points[i].y = transCur(1, 0) * pointFrom->x + transCur(1, 1) * pointFrom->y + transCur(1, 2) * pointFrom->z + transCur(1, 3);
            cloudOut->points[i].z = transCur(2, 0) * pointFrom->x + transCur(2, 1) * pointFrom->y + transCur(2, 2) * pointFrom->z + transCur(2, 3);
            cloudOut->points[i].intensity = pointFrom->intensity;
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                            gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                            gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    {
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    // 从xyzRPY的单独数据变成仿射变换矩阵
    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw = transformIn[2];
        return thisPose6D;
    }

    // 更新transformTobeMapped
    // 预积分可用就用预积分更新位姿
    // 预积分不可用，而Imu可用就用imu更新（因该是预积分因子图重置的时候）
    void updateInitialGuess()
    {

        // save current transformation before any processing
        // 上一刻的slam位姿
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation; // 上一次的imu的Transformation;
        // initialization
        // 如果关键帧还是空的，就用imu的真值作为初始的rotation
        // if (cloudKeyPoses3D->points.empty())
        static bool lastImuPreTransAvailable = false; // 是否有lastImuPreTransformation，第一次当然初始化为false了
        if (!pose_initailized || initial_count < initial_count_num || lose_flag)
        {

            std::lock_guard<std::mutex> lock(mtx);
            transformTobeMapped[0] = cloudInfo.imuRollInit;
            transformTobeMapped[1] = cloudInfo.imuPitchInit;
            transformTobeMapped[2] = cloudInfo.imuYawInit;
            // cout<<"initial pose is"<<initialPose[0]<<","<<initialPose[1]<<","<<initialPose[2]<<endl;
            transformTobeMapped[3] = initialPose[0];
            transformTobeMapped[4] = initialPose[1];
            transformTobeMapped[5] = initialPose[2];

            //是否使用imu的初始朝向yaw作为整个地图的初始朝向 ，false则地图从0开始，true地图以imu的yaw作为初始朝向
            if (!useImuHeadingInitialization)
                transformTobeMapped[2] = 0;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            pose_initailized = true;
            lastImuPreTransAvailable = false;
            return;
        }

        // 预积分可用就用预积分更新位姿
        // use imu pre-integration estimation for pose guess

        static Eigen::Affine3f lastImuPreTransformation; // 上一次的imu预积分Transformation
        if (cloudInfo.odomAvailable == true)
        {

            // IMU预积分的Transformation
            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX, cloudInfo.initialGuessY, cloudInfo.initialGuessZ,
                                                               cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
            if (lastImuPreTransAvailable == false)
            {
                // transBack = 上次的位姿
                lastImuPreTransformation = transBack;
                // lastImuPreTransformation = incrementalOdometryAffineFront;
                lastImuPreTransAvailable = true;
            }
            else
            {
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack; // IMU预积分的两次建图间隔的增量
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);             //当前的imu预积分里程计值
                Eigen::Affine3f transFinal = transTobe * transIncre;                         // imu预积分值
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                  transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                lastImuPreTransformation = transBack; //当前预积分值作为上一次的值
                // lastImuPreTransformation = transFinal;

                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
                return;
            }
        }

        // use imu incremental estimation for pose guess (only rotation)
        // Imu可用就用imu更新(只更新旋转，也只有旋转呀O(∩_∩)O哈哈~)
        if (cloudInfo.imuAvailable == true)
        {
            // 当前imu的Transformation
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);

            // IMU两次建图间隔的增量
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped); //上一刻里程计值
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }
    }

    // 降采样当前scan
    void downsampleCurrentScan()
    {
        // Downsample cloud from current scan
        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();
        cout << "焦点数量是：" << laserCloudCornerLastDSNum << endl;

        laserCloudSurfCurDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfCur);
        downSizeFilterSurf.filter(*laserCloudSurfCurDS);
        laserCloudSurfCurDSNum = laserCloudSurfCurDS->size();
        cout << "面点数量是：" << laserCloudSurfCurDSNum << endl;
    }

    // 更新lidar->map 的变换矩阵
    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    // 角点筛选，判断k+1scan的角点是否和k的地图上的边足够接近，只留下足够接近的点放入laserCloudOriCornerVec，用于匹配
    void cornerOptimization()
    {
        updatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudCornerLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudCornerLastDS->points[i];                                       //当前点在scan坐标系下的坐标
            pointAssociateToMap(&pointOri, &pointSel);                                          // 当前点在map坐标系下的坐标
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis); // 找出距离最近的5个点
            // for(int i=0;i<5;++i){
            //     cout<<i<<":"<<pointSearchSqDis[i]<<endl;
            // }
            if (pointSearchSqDis[0] <= 1.0)
            {
                // Add to the fitness score
                Corner_fitness_score += pointSearchSqDis[0];
                Corner_num++;
            }

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

            // 只有当最远的那个邻域点pointSearchSqDis[4]的距离小于1m时才进行下面的计算
            // 以下部分的计算是在计算点集的协方差矩阵，Zhang Ji的论文中有提到这部分
            // 实际就是PCA
            if (pointSearchSqDis[4] < 0.5)
            {
                // 先求5个样本的平均值
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++)
                {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5;
                cy /= 5;
                cz /= 5;
                // 下面在求矩阵matA1=[ax,ay,az]^t*[ax,ay,az]
                // 更准确地说应该是在求协方差matA1
                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++)
                {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax;
                    a12 += ax * ay;
                    a13 += ax * az;
                    a22 += ay * ay;
                    a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5;
                a12 /= 5;
                a13 /= 5;
                a22 /= 5;
                a23 /= 5;
                a33 /= 5;

                matA1.at<float>(0, 0) = a11;
                matA1.at<float>(0, 1) = a12;
                matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12;
                matA1.at<float>(1, 1) = a22;
                matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13;
                matA1.at<float>(2, 1) = a23;
                matA1.at<float>(2, 2) = a33;
                // 求正交阵的特征值和特征向量
                // 特征值：matD1，特征向量：matV1中
                cv::eigen(matA1, matD1, matV1);
                // 边缘：与较大特征值相对应的特征向量代表边缘线的方向（一大两小，大方向）
                // 以下这一大块是在计算点到边缘的距离，最后通过系数s来判断是否距离很近
                // 如果距离很近就认为这个点在边缘上，需要放到laserCloudOri中
                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1))
                {

                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);
                    // 这边是在求[(x0-x1),(y0-y1),(z0-z1)]与[(x0-x2),(y0-y2),(z0-z2)]叉乘得到的向量的模长
                    // 这个模长是由0.2*V1[0]和点[x0,y0,z0]构成的平行四边形的面积
                    /*
                         |  i       j      k   |
                    axb= | x0-x1  y0-y1  z0-z1 | = [(y0-y1)*(z0-z2)-(y0-y2)*(z0 -z1)]i+[(x0-x1)*(z0-z2)-(x0-x2)*(z0-z1)]j+[(x0-x1)*(y0-y2)-(x0-x2)*(y0-y1)]k
                         | x0-x2  y0-y2  z0-z2 |
                    */
                    float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));
                    // l12表示的是0.2*(||V1[0]||)
                    // 也就是平行四边形一条底的长度
                    float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

                    // 又一次叉乘，算距离直线的方向，除以两个向量的模，相当于归一化
                    float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;
                    // 计算点pointSel到直线（过质心的方向向量）的距离
                    // 距离（高）=平行四边形面积/底边长度
                    float ld2 = a012 / l12;
                    // 如果在最理想的状态的话，ld2应该为0，表示点在直线上
                    // 最理想状态s=1；
                    float s = 1 - 0.9 * fabs(ld2);
                    // coeff代表系数的意思
                    // coff用于保存距离的方向向量
                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;

                    // intensity本质上构成了一个核函数，ld2越接近于1，增长越慢
                    // intensity=(1-0.9*ld2)*ld2=ld2-0.9*ld2*ld2
                    coeff.intensity = s * ld2;
                    // 所以就应该认为这个点是边缘点
                    // s>0.1 也就是要求点到直线的距离ld2要小于1m
                    // s越大说明ld2越小(离边缘线越近)，这样就说明点pointOri在直线上
                    if (s > 0.1)
                    {
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        laserCloudOriCornerFlag[i] = true;
                    }
                }
            }
        }
    }

    // 平面点筛选，判断k+1scan的平面点是否和k的地图上的平面足够接近，只留下足够接近的点放入laserCloudOriSurfVec，用于匹配
    void surfOptimization()
    {
        updatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfCurDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            // 当前点Scan系下
            pointOri = laserCloudSurfCurDS->points[i];
            // 世界坐标系下的点
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
            if (pointSearchSqDis[0] <= 1.0)
            {
                // Add to the fitness score
                Surf_fitness_score += pointSearchSqDis[0];
                Surf_num++;
            }
            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[4] < 1.0)
            {
                for (int j = 0; j < 5; j++)
                {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }
                // matB0是一个5x1的矩阵
                // matB0 = cv::Mat (5, 1, CV_32F, cv::Scalar::all(-1));
                // matX0是3x1的矩阵
                // 求解方程matA0*matX0=matB0
                // 公式其实是在求由matA0中的点构成的平面的法向量matX0
                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                // [pa,pb,pc,pd]=[matX0,pd]
                // 正常情况下（见后面planeValid判断条件），应该是
                // pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                // pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                // pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z = -1
                // 所以pd设置为1
                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                // 对[pa,pb,pc,pd]进行单位化
                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                // 求解后再次检查平面是否是有效平面
                bool planeValid = true;
                for (int j = 0; j < 5; j++)
                {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2)
                    {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid)
                {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
                    // 后面部分相除求的是[pa,pb,pc,pd]与pointSel的夹角余弦值(两个sqrt，其实并不是余弦值)
                    // 这个夹角余弦值越小越好，越小证明所求的[pa,pb,pc,pd]与平面越垂直
                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;
                    // 判断是否是合格平面，是就加入laserCloudOriSurfVec
                    if (s > 0.1)
                    {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    void surfOptimizationScan()
    {

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfCurDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            // 当前点Scan系下
            pointOri = laserCloudSurfCurDS->points[i];
            // 上一帧坐标系下
            pointTransForm(&pointOri, &pointSel, D_T);
            kdtreeLastScan->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
            if (pointSearchSqDis[0] <= 1.0)
            {
                // Add to the fitness score
                Surf_fitness_score += pointSearchSqDis[0];
                Surf_num++;
            }
            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[4] < 1.0)
            {
                for (int j = 0; j < 5; j++)
                {
                    matA0(j, 0) = laserCloudSurfLast->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfLast->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfLast->points[pointSearchInd[j]].z;
                }
                // matB0是一个5x1的矩阵
                // matB0 = cv::Mat (5, 1, CV_32F, cv::Scalar::all(-1));
                // matX0是3x1的矩阵
                // 求解方程matA0*matX0=matB0
                // 公式其实是在求由matA0中的点构成的平面的法向量matX0
                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                // [pa,pb,pc,pd]=[matX0,pd]
                // 正常情况下（见后面planeValid判断条件），应该是
                // pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                // pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                // pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z = -1
                // 所以pd设置为1
                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                // 对[pa,pb,pc,pd]进行单位化
                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                // 求解后再次检查平面是否是有效平面
                bool planeValid = true;
                for (int j = 0; j < 5; j++)
                {
                    if (fabs(pa * laserCloudSurfLast->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfLast->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfLast->points[pointSearchInd[j]].z + pd) > 0.2)
                    {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid)
                {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
                    // 后面部分相除求的是[pa,pb,pc,pd]与pointSel的夹角余弦值(两个sqrt，其实并不是余弦值)
                    // 这个夹角余弦值越小越好，越小证明所求的[pa,pb,pc,pd]与平面越垂直
                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;
                    // 判断是否是合格平面，是就加入laserCloudOriSurfVec
                    if (s > 0.1)
                    {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    // 把laserCloudOriCornerVec和laserCloudOriSurfVec的点放入laserCloudOri
    // 向量系数coeffSelCornerVec和coeffSelSurfVec放入coeffSel
    // TODO 为什么不像lego-loam中一样直接放进去得了，这不是多此一举嘛
    void combineOptimizationCoeffs()
    {
        // combine corner coeffs
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i)
        {
            if (laserCloudOriCornerFlag[i] == true)
            {
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfCurDSNum; ++i)
        {
            if (laserCloudOriSurfFlag[i] == true)
            {
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    // L-M优化transformTobeMapped ,虽然这里转到了camera坐标，但后面又转回来了。所以transformTobeMapped依旧是lidar坐标的
    // 这部分的代码是基于高斯牛顿法的优化，不是zhang ji论文中提到的基于L-M的优化方法
    // 这部分的代码使用旋转矩阵对欧拉角求导，优化欧拉角，不是zhang ji论文中提到的使用angle-axis的优化
    // 优化，使距离最小
    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]); // lidar roll -》camera yaw
        float crz = cos(transformTobeMapped[0]);

        // 可用于匹配的点的数目
        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50)
        { // laser cloud original 点云太少，就跳过这次循环
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++)
        {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;
            // lidar -> camera
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera 在相机坐标系中
            // 求雅克比矩阵中的元素，距离d对roll角度的偏导量即d(d)/d(roll)
            // 更详细的数学推导参看wykxwyc.github.io
            float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;
            // 同上，求解的是对pitch的偏导量
            float ary = ((cry * srx * srz - crz * sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x + ((-cry * crz - srx * sry * srz) * pointOri.x + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;
            // 同上，求解的是对yaw的偏导量
            float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y + ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;

            /*
            在求点到直线的距离时，coeff表示的是如下内容
            [la,lb,lc]表示的是点到直线的垂直连线方向，s是长度
            coeff.x = s * la;
            coeff.y = s * lb;
            coeff.z = s * lc;
            coeff.intensity = s * ld2;

            在求点到平面的距离时，coeff表示的是
            [pa,pb,pc]表示过外点的平面的法向量，s是线的长度
            coeff.x = s * pa;
            coeff.y = s * pb;
            coeff.z = s * pc;
            coeff.intensity = s * pd2;
            */
            // lidar <- camera
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            // 这部分是雅克比矩阵中距离对平移的偏导
            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            // 残差项
            matB.at<float>(i, 0) = -coeff.intensity;
        }
        // 将矩阵由matA转置生成matAt
        // 先进行计算，以便于后边调用 cv::solve求解
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        // 利用高斯牛顿法进行求解，
        // 高斯牛顿法的原型是J^(T)*J * delta(x) = -J*f(x)
        // J是雅克比矩阵，这里是A，f(x)是优化目标，这里是-B(符号在给B赋值时候就放进去了)
        // 通过QR分解的方式，求解matAtA*matX=matAtB，得到解matX
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        // iterCount==0 说明是第一次迭代，需要初始化
        if (iterCount == 0)
        {

            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--)
            {
                if (matE.at<float>(0, i) < eignThre[i])
                {
                    for (int j = 0; j < 6; j++)
                    {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                }
                else
                {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
            pow(matX.at<float>(3, 0) * 100, 2) +
            pow(matX.at<float>(4, 0) * 100, 2) +
            pow(matX.at<float>(5, 0) * 100, 2));
        // 旋转和平移量足够小就停止这次迭代过程
        if (deltaR < 0.05 && deltaT < 0.05)
        {
            return true; // converged
        }
        return false; // keep optimizing
    }

    bool LMOptimizationScan(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        Eigen::Matrix3f Rotation = D_T.block<3, 3>(0, 0);
        Eigen::Vector3f Trans = D_T.block<3, 1>(0, 3);
        Eigen::Vector3f eulerAngle = Rotation.eulerAngles(0, 1, 2);

        float srx = sin(eulerAngle(1));
        float crx = cos(eulerAngle(1));
        float sry = sin(eulerAngle(2));
        float cry = cos(eulerAngle(2));
        float srz = sin(eulerAngle(0)); // lidar roll -》camera yaw
        float crz = cos(eulerAngle(0));

        // 可用于匹配的点的数目
        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50)
        { // laser cloud original 点云太少，就跳过这次循环
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++)
        {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;
            // lidar -> camera
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera 在相机坐标系中
            // 求雅克比矩阵中的元素，距离d对roll角度的偏导量即d(d)/d(roll)
            // 更详细的数学推导参看wykxwyc.github.io
            float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;
            // 同上，求解的是对pitch的偏导量
            float ary = ((cry * srx * srz - crz * sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x + ((-cry * crz - srx * sry * srz) * pointOri.x + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;
            // 同上，求解的是对yaw的偏导量
            float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y + ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;

            /*
            在求点到直线的距离时，coeff表示的是如下内容
            [la,lb,lc]表示的是点到直线的垂直连线方向，s是长度
            coeff.x = s * la;
            coeff.y = s * lb;
            coeff.z = s * lc;
            coeff.intensity = s * ld2;

            在求点到平面的距离时，coeff表示的是
            [pa,pb,pc]表示过外点的平面的法向量，s是线的长度
            coeff.x = s * pa;
            coeff.y = s * pb;
            coeff.z = s * pc;
            coeff.intensity = s * pd2;
            */
            // lidar <- camera
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            // 这部分是雅克比矩阵中距离对平移的偏导
            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            // 残差项
            matB.at<float>(i, 0) = -coeff.intensity;
        }
        // 将矩阵由matA转置生成matAt
        // 先进行计算，以便于后边调用 cv::solve求解
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        // 利用高斯牛顿法进行求解，
        // 高斯牛顿法的原型是J^(T)*J * delta(x) = -J*f(x)
        // J是雅克比矩阵，这里是A，f(x)是优化目标，这里是-B(符号在给B赋值时候就放进去了)
        // 通过QR分解的方式，求解matAtA*matX=matAtB，得到解matX
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        // iterCount==0 说明是第一次迭代，需要初始化
        if (iterCount == 0)
        {

            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--)
            {
                if (matE.at<float>(0, i) < eignThre[i])
                {
                    for (int j = 0; j < 6; j++)
                    {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                }
                else
                {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        eulerAngle(0) += matX.at<float>(0, 0);
        eulerAngle(1) += matX.at<float>(1, 0);
        eulerAngle(2) += matX.at<float>(2, 0);
        Trans(0) += matX.at<float>(3, 0);
        Trans(1) += matX.at<float>(4, 0);
        Trans(2) += matX.at<float>(5, 0);

        Eigen::AngleAxisf rollAngle(AngleAxisf(eulerAngle(0), Vector3f::UnitX()));
        Eigen::AngleAxisf pitchAngle(AngleAxisf(eulerAngle(1), Vector3f::UnitY()));
        Eigen::AngleAxisf yawAngle(AngleAxisf(eulerAngle(2), Vector3f::UnitZ()));

        Eigen::Matrix3f rotation_matrix;
        rotation_matrix = yawAngle * pitchAngle * rollAngle;
        Eigen::Matrix4f tmp = Eigen::Matrix4f::Identity();
        tmp.block<3, 3>(0, 0) = rotation_matrix;
        tmp.block<3, 1>(0, 3) = Trans;
        D_T = tmp;

        float deltaR = sqrt(
            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
            pow(matX.at<float>(3, 0) * 100, 2) +
            pow(matX.at<float>(4, 0) * 100, 2) +
            pow(matX.at<float>(5, 0) * 100, 2));
        // 旋转和平移量足够小就停止这次迭代过程
        if (deltaR < 0.05 && deltaT < 0.05)
        {
            return true; // converged
        }
        return false; // keep optimizing
    }

    // scan2map匹配优化
    void scan2MapOptimization()
    {
        // if (cloudKeyPoses3D->points.empty())
        if (!pose_initailized)
            return;

        //大于最小阈值
        // laserCloudCornerFromMapDSNum是extractSurroundingKeyFrames()函数最后降采样得到的coner点云数
        // laserCloudSurfFromMapDSNum是extractSurroundingKeyFrames()函数降采样得到的surface点云数
        if (laserCloudSurfCurDSNum > surfFeatureMinValidNum)
        {

            // kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);

            // kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            // 用for循环控制迭代次数，最多迭代30次
            for (int iterCount = 0; iterCount < iter_num; iterCount++)
            {
                Corner_fitness_score = 0.0;
                Surf_fitness_score = 0.0;
                Corner_num = 0;
                Surf_num = 0;

                laserCloudOri->clear();
                coeffSel->clear();
                // 在 Qk中选取相邻点集合S，计算S的协方差矩阵M、特征向量E、特征值V。选取边缘线和平面块方式为：
                // 边缘线：V中特征值一大两小，E中大特征值代表的特征向量代表边缘线的方向。
                // 平面块：V中一小两大，E中小特征值对应的特征向量代表平面片的方向。
                // 边缘线或平面块的位置通过穿过S的几何中心来确定。
                cornerOptimization();
                if (Corner_num > 1)
                    Corner_fitness_score = (Corner_fitness_score / (double)Corner_num);
                else
                    Corner_fitness_score = (std::numeric_limits<double>::max());
                surfOptimization();
                if (Surf_num > 1)
                    Surf_fitness_score = (Surf_fitness_score / (double)Surf_num);
                else
                    Surf_fitness_score = (std::numeric_limits<double>::max());

                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true)
                    break;
            }

            // 迭代结束更新相关的转移矩阵
            transformUpdate();
        }
        else
        {
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfCurDSNum);
        }
    }

    void scan2scanOptimization()
    {
        // if (cloudKeyPoses3D->points.empty())
        if (!pose_initailized)
            return;

        //大于最小阈值
        // laserCloudCornerFromMapDSNum是extractSurroundingKeyFrames()函数最后降采样得到的coner点云数
        // laserCloudSurfFromMapDSNum是extractSurroundingKeyFrames()函数降采样得到的surface点云数
        if (laserCloudSurfCurDSNum > surfFeatureMinValidNum)
        {

            kdtreeLastScan->setInputCloud(laserCloudSurfLast);

            // kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            // 用for循环控制迭代次数，最多迭代30次
            for (int iterCount = 0; iterCount < iter_num; iterCount++)
            {
                Corner_fitness_score = 0.0;
                Surf_fitness_score = 0.0;
                Corner_num = 0;
                Surf_num = 0;

                laserCloudOri->clear();
                coeffSel->clear();
                // 在 Qk中选取相邻点集合S，计算S的协方差矩阵M、特征向量E、特征值V。选取边缘线和平面块方式为：
                // 边缘线：V中特征值一大两小，E中大特征值代表的特征向量代表边缘线的方向。
                // 平面块：V中一小两大，E中小特征值对应的特征向量代表平面片的方向。
                // 边缘线或平面块的位置通过穿过S的几何中心来确定。
                // cornerOptimization();
                // if (Corner_num > 1)
                //     Corner_fitness_score= (Corner_fitness_score / (double)Corner_num);
                // else
                //     Corner_fitness_score= (std::numeric_limits<double>::max ());
                surfOptimizationScan();
                if (Surf_num > 1)
                    Surf_fitness_score = (Surf_fitness_score / (double)Surf_num);
                else
                    Surf_fitness_score = (std::numeric_limits<double>::max());

                combineOptimizationCoeffs();

                // TODO
                if (LMOptimizationScan(iterCount) == true)
                    break;
            }
        }
        else
        {
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfCurDSNum);
        }
    }

    // 如果imuAvailable，和imu插值融合roll 和pitch更新transformTobeMapped和incrementalOdometryAffineBack
    // 检查阈值约束，大于阈值的=阈值
    void transformUpdate()
    {
        // if (cloudInfo.imuAvailable == true)
        // {
        //     // 使用imu和transfrom插值得到roll和pitch
        //     if (std::abs(cloudInfo.imuPitchInit) < 1.4)
        //     {
        //         double imuWeight = 0.01;
        //         tf::Quaternion imuQuaternion;
        //         tf::Quaternion transformQuaternion;
        //         double rollMid, pitchMid, yawMid;

        //         // slerp roll
        //         transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
        //         imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
        //         tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);// slerp 差值
        //         transformTobeMapped[0] = rollMid;

        //         // slerp pitch
        //         transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
        //         imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
        //         tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
        //         transformTobeMapped[1] = pitchMid;
        //     }
        // }

        // 差数设的是1000，相当于没有约束
        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        // transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);
        if (transformTobeMapped[5] < -5.0)
            transformTobeMapped[5] = -5.0;
        if (transformTobeMapped[5] > 1)
            transformTobeMapped[5] = 1;
        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    // 最大最小值范围约束
    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    // 判断是否需要保存关键帧，
    // 当RPY角度或者位移大于阈值，则为true
    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty())
            return true;

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll) < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
            abs(yaw) < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    // 添加里程计因子
    void addOdomFactor()
    {
        // 为空添加先验因子
        if (cloudKeyPoses3D->points.empty())
        {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }
        else
        {
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }

    // 对齐gps时间戳添加gps因子
    void addGPSFactor()
    {
        if (gpsQueue.empty())
            return;

        // wait for system initialized and settles down
        if (cloudKeyPoses3D->points.empty()) // 没有关键帧，不添加gps
            return;
        else
        {
            // 关键帧距离太近，不添加
            if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
                return;
        }

        // pose covariance small, no need to correct
        // 位置协方差太小，不添加
        if (poseCovariance(3, 3) < poseCovThreshold && poseCovariance(4, 4) < poseCovThreshold)
            return;

        // last gps position
        static PointType lastGPSPoint;

        // 对齐时间戳
        while (!gpsQueue.empty())
        {
            if (gpsQueue.front().header.stamp.toSec() < timeLaserInfoCur - 0.2)
            {
                // message too old
                gpsQueue.pop_front();
            }
            else if (gpsQueue.front().header.stamp.toSec() > timeLaserInfoCur + 0.2)
            {
                // message too new
                break;
            }
            else
            {
                nav_msgs::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                    continue;

                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                if (!useGpsElevation)
                {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                // GPS not properly initialized (0,0,0)
                // GPS未正确初始化（0,0,0）
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                    continue;

                // Add GPS every a few meters
                //每隔几米增加一次GPS
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;
                if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                    continue;
                else
                    lastGPSPoint = curGPSPoint;

                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);
                break;
            }
        }
    }

    // 添加里程计、gps、闭环因子，并执行gtsam优化，保存优化后的当前关键帧
    // 保存关键帧点云
    // 发布关键帧路径
    void saveKeyFramesAndFactor()
    {
        if (saveFrame() == false)
            return;

        // odom factor
        addOdomFactor();

        // gps factor
        // addGPSFactor();

        // cout << "****************************************************" << endl;
        // gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        // save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        // 添加当前帧
        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
        thisPose6D.roll = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw = latestEstimate.rotation().yaw();
        thisPose6D.timestamp = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();
        // save path for visualization
        // 发布关键帧路径
        // updatePath(thisPose6D);
    }

    //   void saveKeyFramesAndFactor2()
    // {
    //     PointType thisPose3D;
    //     PointTypePose thisPose6D;
    //     // 添加当前帧
    //     thisPose3D.x = transformTobeMapped[3];
    //     thisPose3D.y = transformTobeMapped[4];
    //     thisPose3D.z = transformTobeMapped[5];
    //     thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
    //     // cloudKeyPoses3D->push_back(thisPose3D);

    //     thisPose6D.x = thisPose3D.x;
    //     thisPose6D.y = thisPose3D.y;
    //     thisPose6D.z = thisPose3D.z;
    //     thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
    //     thisPose6D.roll  = transformTobeMapped[0];
    //     thisPose6D.pitch = transformTobeMapped[1];
    //     thisPose6D.yaw   = transformTobeMapped[2];
    //     thisPose6D.timestamp = timeLaserInfoCur;
    //     // cloudKeyPoses6D->push_back(thisPose6D);
    //     // 发布关键帧路径
    //     updatePath(thisPose6D);
    // }

    // 更新关键帧路径信息
    void updatePath(const PointTypePose &pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void updatePath(const float pose[])
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose[3];
        pose_stamped.pose.position.y = pose[4];
        pose_stamped.pose.position.z = pose[5];
        tf::Quaternion q = tf::createQuaternionFromRPY(pose[0], pose[1], pose[2]);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    // 发布优化真实的里程计和不经过优化的增量里程计
    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        laserOdometryROS.pose.covariance[0] = lose_flag;
        laserOdometryROS.pose.covariance[1] = Surf_fitness_score;
        if (isDegenerate)
            laserOdometryROS.pose.covariance[2] = 1;
        else
            laserOdometryROS.pose.covariance[2] = 0;
        pubLaserOdometryGlobal.publish(laserOdometryROS);
        // cout<< "普通xyz："<<transformTobeMapped[3]<<","<<transformTobeMapped[4]<<","<<transformTobeMapped[5]<<endl;
        // Publish TF
        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                                      tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, odometryFrame, lidarFrame);
        br.sendTransform(trans_odom_to_lidar);

        // // Publish odometry for ROS (incremental)
        // static bool lastIncreOdomPubFlag = false;
        // static nav_msgs::Odometry laserOdomIncremental; // incremental odometry msg
        // static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
        // if (lastIncreOdomPubFlag == false)
        // {
        //     lastIncreOdomPubFlag = true;
        //     laserOdomIncremental = laserOdometryROS;
        //     increOdomAffine = trans2Affine3f(transformTobeMapped);
        // } else {
        // D_T = (incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack).matrix(); // 优化前-1*优化后的= 优化的修正量
        //     increOdomAffine = increOdomAffine * affineIncre;
        //     float x, y, z, roll, pitch, yaw;

        //     pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
        //     // cout<< "增量xyz："<<x<<","<<y<<","<<z<<endl;

        //     // cout<< "两者的差值："<<transformTobeMapped[3]-x<<","<<transformTobeMapped[4]-y<<","<<transformTobeMapped[5]-z<<endl;
        //     // if (cloudInfo.imuAvailable == true)
        //     // {
        //     //     if (std::abs(cloudInfo.imuPitchInit) < 1.4)
        //     //     {
        //     //         double imuWeight = 0.01;
        //     //         tf::Quaternion imuQuaternion;
        //     //         tf::Quaternion transformQuaternion;
        //     //         double rollMid, pitchMid, yawMid;
        //     //         transformQuaternion.setRPY(roll, pitch, 0);
        //     //         imuQuaternion.setRPY(cloudInfo.imuRollInit, cloudInfo.imuPitchInit, 0);
        //     //         tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
        //     //         roll = rollMid;
        //     //         pitch = pitchMid;
        //     //         // yaw = yawMid;
        //     //     }
        //     // }
        //     laserOdomIncremental.header.stamp = timeLaserInfoStamp;
        //     laserOdomIncremental.header.frame_id = odometryFrame;
        //     laserOdomIncremental.child_frame_id = "odom_mapping";
        //     laserOdomIncremental.pose.pose.position.x = x;
        //     laserOdomIncremental.pose.pose.position.y = y;
        //     laserOdomIncremental.pose.pose.position.z = z;
        //     laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        // }
        pubLaserOdometryIncremental.publish(laserOdometryROS);
    }

    // 发布一些没什么用的topic
    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish registered high-res raw cloud
        // if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
        // {
        //     pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        //     pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
        //     PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
        //     *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
        //     publishCloud(&pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        // }
        // publish path
        if (pubPath.getNumSubscribers() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }
        // publishCloud(&pubLaserCloudSurround, globalMap, ros::Time::now(), "slam_map");
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_sam");

    mapOptimization MO;

    ROS_INFO("\033[1;32m----> Localization Started.\033[0m");

    std::thread Dynamic_loadMap(&mapOptimization::dynamic_load_map_run, &MO);

    // ros::spin();
    ros::MultiThreadedSpinner spinner(5);
    spinner.spin();

    Dynamic_loadMap.join();

    return 0;
}
