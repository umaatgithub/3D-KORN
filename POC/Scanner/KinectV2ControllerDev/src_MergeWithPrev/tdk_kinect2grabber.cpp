#include "tdk_kinect2grabber.h"

//#include <Kinect.h>

//default constructor

TDK_Kinect2Grabber::TDK_Kinect2Grabber()
{
    mv_imageViewer = boost::make_shared<pcl::visualization::ImageViewer>("TDK image viewer");
    //mv_cloudViewer = boost::make_shared<pcl::visualization::PCLVisualizer>("TDK cloud viewer");
    //PSP

    //start the grabber
    grabber = boost::make_shared<pcl::Kinect2Grabber>();

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );
    grabber->start();

}

void TDK_Kinect2Grabber::getPointCloudFrame(pcl::PointCloud<PointType>::Ptr &cloud)
{
    boost::mutex::scoped_lock lock(mv_mutex );
    cloud = mv_cloud;
}

void TDK_Kinect2Grabber::closeKinect()
{
    grabber->stop();
}
