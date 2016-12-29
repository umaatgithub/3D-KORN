// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

//#pragma once
//#include <pcl/visualization/image_viewer.h>
#include "TDK_Kinect2Controller.h"
#include <QDebug>
#include <QString>
#include <string>

typedef pcl::PointXYZRGB PointType;

int main()
{
    TDK_Kinect2Grabber kinect;
    int x=-1.5;int y=1.6;int z=-1.5;int r=1.5;int g=0.1;int b=3;
    kinect.mf_SetScanBoxLimits(x,y,z,r,g,b);

    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer( "Point Cloud Viewer") );
    viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0, 0.0, 0.0 );

    // Point Cloud
    pcl::PointCloud<PointType>::Ptr cloud;

    //Spin once to initialize window
    viewer->spinOnce();


    pcl::visualization::ImageViewer image_viewer;


    while( !viewer->wasStopped() )
    {
        viewer->spinOnce();

        //if(kinect.hasNewFrame()){
            kinect.mf_getPointCloudFrame(cloud);

            //If cloud already added, update, if not added, add for first time
            if( !viewer->updatePointCloud( cloud, "cloud" ) )
            {
                viewer->addPointCloud( cloud, "cloud" );
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");
                //kinect.mv_imageViewer->spinOnce();
            }


        //Sleep to make polling a little bit more efficient
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    // Stop Kinect
    kinect.mf_closeKinect();

    return 0;
}
