#include "tdk_scanregistration.h"

#include <QDebug>


using namespace std;

TDK_ScanRegistration::TDK_ScanRegistration()
{
    //Empty constructor
    qDebug() << "Has creado una instancia de TDK_ScanRegistration" << endl;

    mv_voxelSideLength = 0.02;
}

bool TDK_ScanRegistration::addNextPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointcloud)
{
    mv_originalPointClouds.push_back(inputPointcloud);

    //Downsample pointcloud and push to Downsampled vector
    mv_downSampledPointClouds.push_back(mf_voxelDownSamplePointCloud(inputPointcloud, mv_voxelSideLength));

    return true;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr TDK_ScanRegistration::getLastOriginalPointcloud()
{
    return *(--mv_originalPointClouds.end());
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
