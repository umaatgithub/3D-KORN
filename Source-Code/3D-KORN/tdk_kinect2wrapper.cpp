#include "tdk_kinect2wrapper.h"

TDK_Kinect2Wrapper::TDK_Kinect2Wrapper(QObject *parent) : QObject(parent)
{
    mv_grabber = boost::make_shared<pcl::Kinect2Grabber>();
    boost::signals2::connection connection = mv_grabber->registerCallback( mv_pointCloudCallback );
}

void TDK_Kinect2Wrapper::startKinect()
{
    mv_grabber->start();
}

void TDK_Kinect2Wrapper::stopKinect()
{

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr TDK_Kinect2Wrapper::getMv_cloud() const
{
    return mv_cloud;
}

void TDK_Kinect2Wrapper::setMv_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &value)
{
    mv_cloud = value->makeShared();
    emit signalCloudUpdated();
}
