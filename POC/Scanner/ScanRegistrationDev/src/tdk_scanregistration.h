#ifndef TDK_SCANREGISTRATION_H
#define TDK_SCANREGISTRATION_H

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

class TDK_ScanRegistration
{
public:
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;

    TDK_ScanRegistration();
    bool addNextPointCloud(const PointCloudT::Ptr &inputPointcloud);
    PointCloudT::Ptr getLastOriginalPointcloud();

private:
    //Configuration parameters
    float mv_voxelSideLength;

    //Member variables
    vector<PointCloudT::Ptr> mv_originalPointClouds;
    vector<PointCloudT::Ptr> mv_downSampledPointClouds;

    //Private Member functions
    PointCloudT::Ptr mf_voxelDownSamplePointCloud(const PointCloudT::Ptr &cloud_in, const float &voxelSideLength);

};

#endif // TDK_SCANREGISTRATION_H
