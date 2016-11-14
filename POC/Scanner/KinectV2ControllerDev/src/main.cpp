// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

//#pragma once

#include "tdk_kinect2grabber.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <QDebug>
#include <QString>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <string>

typedef pcl::PointXYZRGB PointType;

int main( int argc, char* argv[] )
{
    TDK_Kinect2Grabber kinect;


    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer( "Point Cloud Viewer") );
    viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0, 0.0, 0.0 );

    // Point Cloud
    pcl::PointCloud<PointType>::Ptr cloud;
    
    //Spin once to initialize window
    viewer->spinOnce();

    while( !viewer->wasStopped() ){
        //if(kinect.hasNewFrame()){
            kinect.getPointCloudFrame(cloud);

            //If cloud already added, update, if not added, add for first time
            if( !viewer->updatePointCloud( cloud, "cloud" ) ){
                viewer->addPointCloud( cloud, "cloud" );
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");
            }
            // Update Viewer
            viewer->spinOnce();
    //    }
        //Sleep to make polling a little bit more efficient
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    // Stop Kinect
    //kinect.close();

    return 0;
}
