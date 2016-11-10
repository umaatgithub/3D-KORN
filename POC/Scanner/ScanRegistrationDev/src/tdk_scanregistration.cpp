#include "tdk_scanregistration.h"

#include <QDebug>


using namespace std;

TDK_ScanRegistration::TDK_ScanRegistration()
{
    //Empty constructor
    qDebug() << "Has creado una instancia de TDK_ScanRegistration" << endl;

    mv_voxelSideLength = 0.02;
    mv_FeatureRadiusSearch=0.05;
}

bool TDK_ScanRegistration::addNextPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointcloud)
{
      mv_originalPointClouds.push_back(inputPointcloud);

      //Approach 1 SAC ICP
      mf_processVoxelSacIcp();

    return true;

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr TDK_ScanRegistration::getLastOriginalPointcloud()
{
    return *(--mv_downSampledPointClouds.end());
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr TDK_ScanRegistration::mf_voxelDownSamplePointCloud(const PointCloudT::Ptr &cloud_in, const float &voxelSideLength)
{
    PointCloudT::Ptr downSampledPointCloud(new PointCloudT);

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setLeafSize (voxelSideLength, voxelSideLength, voxelSideLength);

    vg.setInputCloud (cloud_in);
    vg.filter (*downSampledPointCloud);

    return downSampledPointCloud;
}

pcl::PointCloud<pcl::Normal>::Ptr TDK_ScanRegistration::mf_computeNormals(const PointCloudT::Ptr &cloud_in, const float &searchRadius)
{
    SurfaceNormalsT::Ptr normals(new SurfaceNormalsT);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setInputCloud (cloud_in);
    //norm_est.setSearchMethod (new pcl::search::KdTree<pcl::PointXYZRGB>);
    norm_est.setRadiusSearch (searchRadius);
    norm_est.compute (*normals);
    return normals;
}

bool TDK_ScanRegistration::mf_processVoxelSacIcp()
{
    //Downsample pointcloud and push to Downsampled vector
    mv_downSampledPointClouds.push_back(mf_voxelDownSamplePointCloud(mv_originalPointClouds.back(), mv_voxelSideLength));
    mv_downSampledNormals.push_back(mf_computeNormals(mv_downSampledPointClouds.back(),mv_FeatureRadiusSearch));
    mv_downSampledFeatures.push_back(mf_computeLocalFPFH33Features(mv_downSampledPointClouds.back(), mv_downSampledNormals.back(),mv_FeatureRadiusSearch));


    //qDebug() << "Donwsampled Point Cloud Dimension is " << (--mv_downSampledPointClouds.end())->get()->points.size()  <<endl;
    //qDebug() << "features Dimension is " << mv_downSampledFeatures.back().get()->points.size()  <<endl;

    return true;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr
TDK_ScanRegistration::mf_computeLocalFPFH33Features (const PointCloudT::Ptr &cloud_in, const SurfaceNormalsT::Ptr &normal_in, const float &searchRadius)
{
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features = pcl::PointCloud<pcl::FPFHSignature33>::Ptr (new pcl::PointCloud<pcl::FPFHSignature33>);

    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud (cloud_in);
    fpfh_est.setInputNormals (normal_in);
    fpfh_est.setRadiusSearch (searchRadius);
    fpfh_est.compute (*features);

    return features;
}

