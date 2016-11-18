#include "tdk_kinect2wrapper.h"

TDK_Kinect2Wrapper::TDK_Kinect2Wrapper(QObject *parent) : QObject(parent)
{
    mv_grabber = boost::make_shared<pcl::Kinect2Grabber>();

    // Register Callback Function
    boost::signals2::connection connection = mv_grabber->registerCallback( mv_pointCloudCallback );
}

void TDK_Kinect2Wrapper::startKinect()
{
    mv_grabber->start();
}

pcl::PointCloud::ConstPtr TDK_Kinect2Wrapper::getMv_cloud() const
{
    return mv_cloud;
}

void TDK_Kinect2Wrapper::setMv_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &value)
{
    qDebug("Setter fn : Inside setter fn");
    mv_cloud = value->makeShared();
    qDebug("Setter fn : Point cloud set -> "+mv_cloud->size());
    emit signalCloudUpdated(mv_cloud);
    qDebug("Setter fn : Emitted signal");
}
