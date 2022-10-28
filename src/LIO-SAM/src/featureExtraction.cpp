// TODO 没有地面分割了,也没有了角点次角点，平面点次平面点的区别，也没有去除外点

#include "utility.h"
#include "lio_sam/cloud_info.h"
 
//曲率值和序号
struct smoothness_t{ 
    float value;
    size_t ind;
};

//曲率值对比仿函数
struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

class FeatureExtraction : public ParamServer
{

public:

    ros::Subscriber subLaserCloudInfo;

    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubCornerPoints;
    ros::Publisher pubSurfacePoints;

    pcl::PointCloud<PointType>::Ptr extractedCloud; // 输入的去畸变后的点云
    pcl::PointCloud<PointType>::Ptr cornerCloud;  // 角点点云
    pcl::PointCloud<PointType>::Ptr surfaceCloud; // 降采样后的平面点云

    pcl::VoxelGrid<PointType> downSizeFilter;

    lio_sam::cloud_info cloudInfo; //存储收到topic的cloud_info
    std_msgs::Header cloudHeader;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;// 是否已经被挑选出来，即去掉已经筛选过的点的标志 1为已经筛选过 0为还未被筛选
    int *cloudLabel;  // 点云类型标记 初始化为0，平面点为-1，角点为1 ，// TODO 其实好像也没什么用啊反正都已经放到对应的队列进去了，这个根本没用嘛
		                        	

    FeatureExtraction()
    {
        subLaserCloudInfo = nh.subscribe<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/feature/cloud_info", 1);
        pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_corner", 1);
        pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_surface", 1);
        
        initializationValue();
    }

    // 初始化各种参数
    void initializationValue()
    {
        // 提前分配空间
        cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

        // 设置降采样叶子尺寸（0.2）
        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());

        cloudCurvature = new float[N_SCAN*Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
        cloudLabel = new int[N_SCAN*Horizon_SCAN];
    }

    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn)
    {
        cloudInfo = *msgIn; // new cloud info
        cloudHeader = msgIn->header; // new cloud header
        pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud); // new cloud for extraction

        //计算所有点的算曲率
        calculateSmoothness();
        
        // 剔除被遮挡的点不参与特征提取
        markOccludedPoints();

        //对于scan中的角点和面点进行提取
        extractFeatures();

        publishFeatureCloud();
    }

    // 计算所有点的曲率
    // TODO 其实可以直接在rangeMat里算，这样算出来的每条线的边界点怎么办
    void calculateSmoothness()
    {
        int cloudSize = extractedCloud->points.size();
        for (int i = 5; i < cloudSize - 5; i++)
        {
            // 前后各5个点与当前点的range差
            float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                            + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                            + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                            + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                            + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                            + cloudInfo.pointRange[i+5];            

            // 曲率
            cloudCurvature[i] = diffRange*diffRange;//diffX * diffX + diffY * diffY + diffZ * diffZ;

            cloudNeighborPicked[i] = 0; // 初始化赋值
            cloudLabel[i] = 0; // 初始化赋值
            // cloudSmoothness for sorting
            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    // 剔除被遮挡的点不参与特征提取
    // 相邻两点range差距大则设为不进行特征提取的状态，差距过大，周围点也设置为不进行特征提取的状态
    // 标记为1代表这些点已经被筛选，不再参与特征点的提取
    
    void markOccludedPoints()
    {
        int cloudSize = extractedCloud->points.size();
        // mark occluded points and parallel beam points
        for (int i = 5; i < cloudSize - 6; ++i)
        {
            // occluded points
            float depth1 = cloudInfo.pointRange[i];
            float depth2 = cloudInfo.pointRange[i+1];
            //两个点的列差值
            int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));
            // 两个点的水平距离分明很近，但是深度却差的很大，说明这两个点之间存在遮挡
            if (columnDiff < 10){ // 10 pixel diff in range image
                // 哪边深度深，哪边被标记为已经筛选过，即不进行特征提取
                if (depth1 - depth2 > 0.3){ 
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }else if (depth2 - depth1 > 0.3){
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }
            // parallel beam
            // 平行光束
            float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
            float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));
            // 两个相邻光束的range差的很大就可以认为是平行光速
            if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
                cloudNeighborPicked[i] = 1;
            
            // 每一行的前5个和后5个不参与特征提取
            // 后面算曲率的时候已经去掉了
            // if(cloudInfo.pointRowInd[i+5]!= cloudInfo.pointRowInd[i] || cloudInfo.pointRowInd[i-5]!= cloudInfo.pointRowInd[i])
            //     cloudNeighborPicked[i] = 1;

        }
    }

    // void extractFeatures()进行特征抽取，然后分别保存到cornerPointsSharp等等队列中去。
    // 函数首先清空了cornerCloud,surfaceCloud
    // 然后对cloudSmoothness队列中sp到ep之间的点的曲率数据进行从小到大的排列。
    // 曲率最大的20个点作为角点cornerCloud，满足平面阈值的点都作为平面点surfaceCloudScan
    // 最后，因为平面点云太多时，计算量过大，因此需要对点云进行下采样surfaceCloudScanDS，减少计算量。
    // 下采样之后的点作为surfaceCloud
    void extractFeatures()
    {
        cornerCloud->clear();
        surfaceCloud->clear();

        // 一个水平线束的平面点 和其降采样
        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());


        for (int i = 0; i < N_SCAN; i++)
        {
            surfaceCloudScan->clear();

            
            for (int j = 0; j < area_num; j++)
            {
                // 和loam中一样把点云分成了6段分别处理，确保特征点提取均匀
                // 这里就是差值分成了6块区域
                int sp = (cloudInfo.startRingIndex[i] * (area_num - j) + cloudInfo.endRingIndex[i] * j) / area_num;
                int ep = (cloudInfo.startRingIndex[i] * (area_num - 1 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / area_num - 1;

                if (sp >= ep)
                    continue;

                 // 按照曲率从小到大排序
                std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());


                int largestPickedNum = 0; // 被提取特征点的数量
                for (int k = ep; k >= sp; k--)
                {
                    // 因为上面对cloudSmoothness进行了一次从小到大排序，所以ind不一定等于k了
                    int ind = cloudSmoothness[k].ind;

                    if (cloudNeighborPicked[ind] == 0   // 还未被筛选
                        && cloudCurvature[ind] > edgeThreshold) //大于边的阈值
                    {

                        if(k%2 != 0) continue;
                        largestPickedNum++;
                        if (largestPickedNum <= PickedCornerNum){ // 曲率最大的20个点作为角点  //注意这里的k是倒着来的 定位5=个点，给规划建图20
                            cloudLabel[ind] = 1; //角点标记为1
                            cornerCloud->push_back(extractedCloud->points[ind]);
                        } else {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1; //标记为已经筛选过
                        // 把领域的点也标记为已经筛选过的点，应该是为了防止太密集吧
                        for (int l = 1; l <= 5; l++)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

             
                //提取所有满足阈值的平面点
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                    {

                        cloudLabel[ind] = -1;
                        cloudNeighborPicked[ind] = 1;

                        for (int l = 1; l <= 5; l++) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }
                int largestPickedSurfNum = 0; // 被提取特征点的数量
              
                for (int k = sp; k <= ep; k++)
                {
                    if(k%  skipSurf != 0) continue;  
                    largestPickedSurfNum++;
                    if (largestPickedSurfNum <= PickedSurfNum){ // 曲率最大的20个点作为角点  //注意这里的k是倒着来的 定位10=个点给规划建图100
                        if (cloudLabel[k] <= 0){ 
                            surfaceCloudScan->push_back(extractedCloud->points[k]);
                        }
                    } else {
                        break;
                    }
                 
                }
            }

            surfaceCloudScanDS->clear();
            downSizeFilter.setInputCloud(surfaceCloudScan);
            downSizeFilter.filter(*surfaceCloudScanDS);

            *surfaceCloud += *surfaceCloudScanDS;
        }
    }

    // 释放cloudInfo中下一步不需要用到的内容降低topic的通信负担
    void freeCloudInfoMemory()
    {
        cloudInfo.startRingIndex.clear();
        cloudInfo.endRingIndex.clear();
        cloudInfo.pointColInd.clear();
        cloudInfo.pointRange.clear();
    }

    // 发布cloud_info,包含角点点云、面点云和imu相关信息
    void publishFeatureCloud()
    {
        // free cloud info memory
        freeCloudInfoMemory();
        // save newly extracted features
        cloudInfo.cloud_corner  = publishCloud(&pubCornerPoints,  cornerCloud,  cloudHeader.stamp, lidarFrame);
        cloudInfo.cloud_surface = publishCloud(&pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
        // publish to mapOptimization
        pubLaserCloudInfo.publish(cloudInfo);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    FeatureExtraction FE;

    ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");
   
    ros::spin();

    return 0;
}