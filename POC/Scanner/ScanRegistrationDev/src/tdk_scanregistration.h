#ifndef TDK_SCANREGISTRATION_H
#define TDK_SCANREGISTRATION_H

#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/boundary.h>


using namespace std;

class TDK_ScanRegistration
{
public:
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormalsT;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeaturesT;

    TDK_ScanRegistration();
    bool addNextPointCloud(const PointCloudT::Ptr &inputPointcloud);
    PointCloudT::Ptr getLastOriginalPointcloud();
    vector<PointCloudT::Ptr>* mf_getOriginalPointClouds();
    vector<PointCloudT::Ptr>* mf_getAlignedPointClouds();

private:
    //General Approach
    float mv_FeatureRadiusSearch;

    vector<PointCloudT::Ptr> mv_originalPointClouds;
    vector<PointCloudT::Ptr> mv_downSampledPointClouds;
    vector<PointCloudT::Ptr> mv_alignedPointClouds;
    vector<PointCloudT::Ptr> mv_alignedDownSampledPointClouds;
    vector<Eigen::Matrix4f> mv_transformationMatrixAlignment;

    vector<SurfaceNormalsT::Ptr> mv_downSampledNormals;
    SurfaceNormalsT::Ptr mf_computeNormals(const PointCloudT::Ptr &cloud_in, const float &searchRadius);

    vector<LocalFeaturesT::Ptr> mv_downSampledFeatures;

    //Approach 1: Voxel + SAC + ICP
    float mv_voxelSideLength;
    float mv_SAC_MinSampleDistance;
    float mv_SAC_MaxCorrespondenceDistance;
    int mv_SAC_MaximumIterations;

    float mv_ICP_MaxCorrespondenceDistance;

    bool mf_processVoxelSacIcp();
    PointCloudT::Ptr mf_voxelDownSamplePointCloud(const PointCloudT::Ptr &cloud_in, const float &voxelSideLength);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr mf_computeLocalFPFH33Features (const PointCloudT::Ptr &cloud_in, const SurfaceNormalsT::Ptr &normal_in, const float &searchRadius);
    void mf_sampleConsensusInitialAlignment();
    void mf_iterativeClosestPointFinalAlignment();

    //Approach 2: Correspondent Keypoints + SAC + ICP
    double mv_ISS_SalientRadius;
    double mv_ISS_NonMaxRadius;
    double mv_ISS_Gamma21;
    double mv_ISS_Gamma32;
    double mv_ISS_MinNeighbors;
    int mv_ISS_Threads;

    bool TDK_ScanRegistration::mf_processCorrespondencesSVDICP();


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mf_computeISS3DKeyPoints(const PointCloudT::Ptr &cloud_in,
        const double &SalientRadius, const double &NonMaxRadius, const double &Gamma21, const double &Gamma32 , const double &MinNeighbors, const int &Threads);
};

#endif // TDK_SCANREGISTRATION_H
