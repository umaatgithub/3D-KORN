#ifndef TDK_SCANREGISTRATION_H
#define TDK_SCANREGISTRATION_H

#include <math.h>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

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
#include <pcl/point_types_conversion.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>

using namespace std;

void PointCloudXYZRGBtoXYZ(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in, pcl::PointCloud<pcl::PointXYZ>::Ptr &out);

class TDK_ScanRegistration
{
public:
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormalsT;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeaturesT;

    TDK_ScanRegistration();

    bool addNextPointCloud(const PointCloudT::Ptr &inputPointcloud);
    bool addNextPointCloud(const PointCloudT::Ptr &inputPointcloud, float degreesRotatedY);

    PointCloudXYZ::Ptr getLastDownSampledPointcloud();
    vector<PointCloudT::Ptr>* mf_getOriginalPointClouds();
    vector<PointCloudT::Ptr>* mf_getAlignedPointClouds();

    void setMv_SVD_MaxDistance(double value);
    void setMv_ICP_MaxCorrespondenceDistance(float value);
    void setMv_ISS_resolution(double value);

    void setMv_scannerCenterRotation(const pcl::PointXYZ &value);

private:
    //General Approach
    bool mv_scannerCenterRotationSet;
    pcl::PointXYZ mv_scannerCenterRotation;
    float mv_accumulatedRotation;

    vector<PointCloudT::Ptr> mv_originalPointClouds;
    vector<PointCloudXYZ::Ptr> mv_downSampledPointClouds;
    vector<PointCloudXYZ::Ptr> mv_alignedDownSampledPointClouds;
    vector<Eigen::Matrix4f> mv_transformationMatrixAlignment;
    vector<PointCloudT::Ptr> mv_alignedOriginalPointClouds;


    //Approach 1: Voxel + SAC + ICP
    float mv_FeatureRadiusSearch;
    float mv_voxelSideLength;
    float mv_SAC_MinSampleDistance;
    float mv_SAC_MaxCorrespondenceDistance;
    int mv_SAC_MaximumIterations;

    float mv_ICP_MaxCorrespondenceDistance;

    vector<SurfaceNormalsT::Ptr> mv_downSampledNormals;
    SurfaceNormalsT::Ptr mf_computeNormals(const PointCloudXYZ::Ptr &cloud_in, const float &searchRadius);

    vector<LocalFeaturesT::Ptr> mv_downSampledFeatures;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr mf_computeLocalFPFH33Features (const PointCloudXYZ::Ptr &cloud_in, const SurfaceNormalsT::Ptr &normal_in, const float &searchRadius);

    PointCloudXYZ::Ptr mf_voxelDownSamplePointCloud(const PointCloudT::Ptr &cloud_in, const float &voxelSideLength);

    void mf_sampleConsensusInitialAlignment();
    void mf_iterativeClosestPointFinalAlignment();
    bool mf_processVoxelSacIcp();

    //Approach 2: Correspondent Keypoints + SAC + ICP
    double mv_ISS_resolution;
    double mv_ISS_SalientRadius;
    double mv_ISS_NonMaxRadius;
    double mv_ISS_Gamma21;
    double mv_ISS_Gamma32;
    double mv_ISS_MinNeighbors;
    int mv_ISS_Threads;

    double mv_SVD_MaxDistance;
    vector<pcl::CorrespondencesPtr> mv_downsampledCorrespondences;


    pcl::PointCloud<pcl::PointXYZ>::Ptr mf_computeISS3DKeyPoints(const PointCloudT::Ptr &cloud_in,
        const double &SalientRadius, const double &NonMaxRadius, const double &Gamma21, const double &Gamma32 , const double &MinNeighbors, const int &Threads);
    pcl::CorrespondencesPtr mf_estimateCorrespondencesRejection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2, const double &max_distance, pcl::PointCloud<pcl::Normal>::Ptr &normals1, pcl::PointCloud<pcl::Normal>::Ptr &normals2);
    void mf_SVDInitialAlignment();

    bool mf_processCorrespondencesSVDICP();
    bool mf_processVoxelIcp();
};

#endif // TDK_SCANREGISTRATION_H
