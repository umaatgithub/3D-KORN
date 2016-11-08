#ifndef TDK_SCANREGISTRATION_H
#define TDK_SCANREGISTRATION_H

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

class TDK_ScanRegistration
{
public:
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;

    TDK_ScanRegistration();
    bool addNextPointCloud(const PointCloudT::Ptr &inputPointcloud);
    PointCloudT::Ptr getLastOriginalPointcloud();

private:
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormalsT;
    //Configuration parameters
    float mv_voxelSideLength;
    float mv_FeatureRadiusSearch;


    //Member variables
    vector<PointCloudT::Ptr> mv_originalPointClouds;
    vector<PointCloudT::Ptr> mv_downSampledPointClouds;
    vector<SurfaceNormalsT::Ptr> mv_downSampledNormals;

    //Private Member functions
    PointCloudT::Ptr mf_voxelDownSamplePointCloud(const PointCloudT::Ptr &cloud_in, const float &voxelSideLength);
    SurfaceNormalsT::Ptr mf_computeNormals(const PointCloudT::Ptr &cloud_in, const float &searchRadius);
};

#endif // TDK_SCANREGISTRATION_H
