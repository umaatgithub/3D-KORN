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
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/point_types_conversion.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/registration/elch.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/PCLPointCloud2.h>

using namespace std;

void PointCloudXYZRGBtoXYZ(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &out
        );
double computeCloudResolution (
        const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud
        );

class TDK_ScanRegistration
{
public:
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormalsT;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeaturesT;

    TDK_ScanRegistration();

    //Input
    bool addNextPointCloud(const PointCloudT::Ptr &inputPointcloud);
    bool addNextPointCloud(const PointCloudT::Ptr &inputPointcloud, float degreesRotatedY);

    //Ouput
    PointCloudXYZ::Ptr getLastDownSampledPointcloud();
    PointCloudT::Ptr mf_getMergedAlignedPC();
    PointCloudT::Ptr mf_getMergedPostRegisteredPC();
    vector<PointCloudT::Ptr>* mf_getOriginalPointClouds();
    vector<PointCloudT::Ptr>* mf_getAlignedPointClouds();

    //Configuration
    void setMv_SVD_MaxDistance(double value);
    void setMv_ICP_MaxCorrespondenceDistance(float value);
    void setMv_scannerCenterRotation(const pcl::PointXYZ &value);
    void setMv_ICPPost_MaxCorrespondanceDistance(float value);

private:
    //General Approach
    bool mv_scannerCenterRotationSet;
    pcl::PointXYZ mv_scannerCenterRotation;
    float mv_accumulatedRotation;

    float mv_normalRadiusSearch;

    float mv_voxelSideLength;

    double mv_SVD_MaxDistance;

    float mv_ICP_MaxCorrespondenceDistance;

    float mv_ICPPost_MaxCorrespondanceDistance;

    vector<PointCloudT::Ptr> mv_originalPCs;
    vector<PointCloudT::Ptr> mv_originalDenoisedPCs;
    vector<PointCloudXYZ::Ptr> mv_downSampledPCs;
    vector<SurfaceNormalsT::Ptr> mv_downSampledNormals;
    vector<pcl::CorrespondencesPtr> mv_downsampledCorrespondences;
    vector<PointCloudXYZ::Ptr> mv_alignedDownSampledPCs;
    vector<Eigen::Matrix4f> mv_transformationMatrices;
    vector<PointCloudT::Ptr> mv_alignedOriginalPCs;

    //Utility functions
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    mf_denoisePointCloud(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
            const float meanK=8,
            const float std_dev=2.5
            );

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    mf_denoisePointCloud(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
            const float meanK=8,
            const float std_dev=2.5
            );

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    mf_voxelDownSamplePointCloud(
            const PointCloudT::Ptr &cloud_in,
            const float &voxelSideLength
            );

    SurfaceNormalsT::Ptr
    mf_computeNormals(
            const PointCloudXYZ::Ptr &cloud_in,
            const float &searchRadius
            );

    pcl::CorrespondencesPtr
    mf_estimateCorrespondences(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2,
            const pcl::PointCloud<pcl::Normal>::Ptr &normals1,
            const pcl::PointCloud<pcl::Normal>::Ptr &normals2,
            const double &max_distance
            );

    template <typename PointT>
    boost::shared_ptr<pcl::PointCloud<PointT>>
    mf_iterativeClosestPointFinalAlignment(
            const boost::shared_ptr<pcl::PointCloud<PointT>> &source,
            const boost::shared_ptr<pcl::PointCloud<PointT>> &target,
            const float &maxCorrespondenceDistance,
            Eigen::Matrix4f &icpTransformation
            );

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    mf_SVDInitialAlignment(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
            pcl::CorrespondencesPtr correspondences,
            Eigen::Matrix4f &transformation_matrix
            );

    bool
    mf_processCorrespondencesSVDICP();
};

#endif // TDK_SCANREGISTRATION_H
