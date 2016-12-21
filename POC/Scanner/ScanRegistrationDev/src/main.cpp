// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <boost/thread/thread.hpp>
#include <QDebug>
#include <limits>
#include "tdk_scanregistration.h"

int
main (int argc, char** argv)
{
    int numPointclouds = 16;


    TDK_ScanRegistration scanRegistrator;

    // x center = 0.087, z_center = 1.4
    //Pamir is 0.2, 0, 2.25 + 0.3 (with rotation compensaiton
    pcl::PointWithViewpoint scannerCenter(0.18, 0.0, 2.12, 21, 0, 0); //2.054 obtained from max_z in center slice
    scanRegistrator.setScannerCenter(scannerCenter);


    //Load pc
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    for (int i = 0; i < numPointclouds; i++) {
        cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPLYFile("../pamir_stand" + to_string(i) +".ply", *cloud);

        //add to myRegistrator
        scanRegistrator.addNextPointCloud(cloud, -22.5);
    }

    // Initializing point cloud visualizer

    //add to viewer
    qDebug() << "Post processing pointcloud...";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedPC = scanRegistrator.mf_getMergedPostRegisteredPC();

    //pcl::io::savePLYFileBinary("pamir.ply", *mergedPC);
    //    qDebug() << "Post processing finished...";

    boost::shared_ptr<pcl::visualization::PCLVisualizer>
            viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0.9, 0.9, 0.9);

    viewer->addPointCloud( mergedPC, "pc" );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pc");


    //vector<TDK_ScanRegistration::PointCloudT::Ptr> * alignedPCs = scanRegistrator.mf_getOriginalPointClouds();
    //vector<TDK_ScanRegistration::PointCloudT::Ptr> * alignedPCs = scanRegistrator.mf_getAlignedPointClouds();

//    for (int i = 0; i < (*alignedPCs).size(); ++i) {
//        viewer->addPointCloud( (*alignedPCs)[i], "pc"+to_string(i) );
//        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pc"+to_string(i));
//    }

    // Starting visualizer
    viewer->addCoordinateSystem (0.2, "global");

    // Wait until visualizer window is closed.
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return (0);
}
