#include "tdk_kinect2grabber.h"

//#include <Kinect.h>

//default constructor
TDK_Kinect2Grabber::TDK_Kinect2Grabber()
{
    //start the grabber
    grabber = boost::make_shared<pcl::Kinect2Grabber>();

    //call-back updates a member variable with a new point cloud when available
    //          also projects/throws the cloud into the display window

    boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =
            [this]( const pcl::PointCloud<PointType>::ConstPtr& ptr ){
                boost::mutex::scoped_lock lock(mv_mutex );

                /* Point Cloud Processing */
                mv_cloud = ptr->makeShared();

            };

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );

    grabber->start();

}

void TDK_Kinect2Grabber::getPointCloudFrame(pcl::PointCloud<PointType>::Ptr &cloud)
{
    boost::mutex::scoped_lock lock(mv_mutex );
    //cloud = mv_cloud;
}
