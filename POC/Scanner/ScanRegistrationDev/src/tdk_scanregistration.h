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
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormalsT;

    TDK_ScanRegistration();
    bool addNextPointCloud(const PointCloudT::Ptr &inputPointcloud);
    PointCloudT::Ptr getLastOriginalPointcloud();

private:
    //General Approach
    float mv_FeatureRadiusSearch;

    vector<PointCloudT::Ptr> mv_originalPointClouds;
    vector<PointCloudT::Ptr> mv_downSampledPointClouds;

    vector<SurfaceNormalsT::Ptr> mv_downSampledNormals;
    SurfaceNormalsT::Ptr mf_computeNormals(const PointCloudT::Ptr &cloud_in, const float &searchRadius);


    //Approach 1: Voxel + SAC + ICP
    float mv_voxelSideLength;
    PointCloudT::Ptr mf_voxelDownSamplePointCloud(const PointCloudT::Ptr &cloud_in, const float &voxelSideLength);

    //Approach 2: Correspondent Keypoints + SAC + ICP

};

#endif // TDK_SCANREGISTRATION_H
