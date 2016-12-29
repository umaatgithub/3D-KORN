#include "TDK_Kinect2Controller.h"

TDK_Kinect2Grabber::TDK_Kinect2Grabber()
{
    //start the grabber
    mv_grabber = boost::make_shared<pcl::Kinect2Grabber>();

    // Register Callback Function

    boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> mv_pointCloudCallback =
            [this]( const pcl::PointCloud<PointType>::ConstPtr& ptr )
    {
        boost::mutex::scoped_lock lock(mv_mutex );

        /* Point Cloud Processing */
        mv_cloud = ptr->makeShared();

    };

    boost::signals2::connection connection = mv_grabber->registerCallback( mv_pointCloudCallback );

    //pcl::visualization::ImageViewer mv_imageViewer;

    mv_grabber->start();
    //mv_imageViewer = boost::make_shared<pcl::visualization::ImageViewer>("TDK image viewer");
}


void TDK_Kinect2Grabber::mf_getPointCloudFrame(pcl::PointCloud<PointType>::Ptr &cloud)
{
    boost::mutex::scoped_try_lock lock( mv_mutex );
    cloud = mv_cloud;
}

void TDK_Kinect2Grabber::mf_SetScanBoxLimits(int& x_min,int& x_max,int& y_min,int& y_max,int& z_min,int& z_max)
{
    mv_grabber->SetScanBoxLimits( x_min, x_max, y_min, y_max, z_min, z_max);
}

void TDK_Kinect2Grabber::mf_closeKinect()
{
    mv_grabber->stop();
}
