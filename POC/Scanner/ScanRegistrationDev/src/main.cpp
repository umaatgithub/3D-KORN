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

typedef pcl::PointXYZRGBA PointType;
const float bad_point = std::numeric_limits<float>::quiet_NaN();

int
main (int argc, char** argv)
{
    int numPointclouds = (int) atoi(argv[5]);


    pcl::PointXYZ scannerCenter((float)atof(argv[1]), (float)atof(argv[2]), (float)atof(argv[3]));


    TDK_ScanRegistration scanRegistrator;
    //pcl::PointXYZ scannerCenter(0.1, 0.0, 1.8); //2.054 obtained from max_z in center slice
    scanRegistrator.setMv_scannerCenterRotation(scannerCenter);

    //Load pc
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    for (int i = 1; i <= numPointclouds; ++i) {
        cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPLYFile("../alb" + to_string(i) +".ply", *cloud);
        //add to myRegistrator
        scanRegistrator.addNextPointCloud(cloud, (float)atof(argv[4])); //33

        //Estimate center of rot
//        float min_z = 10000;
//        for (int j = 0; j < cloud->points.size(); ++j) {
//           if( abs(cloud->points[j].y) < 0.05 && min_z > cloud->points[j].z){
//                   min_z = cloud->points[j].z;
//           }
//        }
//        qDebug() << "Min Z: " << min_z;

        qDebug() << "Number of keypoints = " << scanRegistrator.getLastDownSampledPointcloud()->points.size();
    }


    // Initializing point cloud visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
            viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    //add to viewer
    vector<TDK_ScanRegistration::PointCloudT::Ptr> * alignedPCs = scanRegistrator.mf_getAlignedPointClouds();
    //vector<TDK_ScanRegistration::PointCloudT::Ptr> * alignedPCs = scanRegistrator.mf_getOriginalPointClouds();

    for (int i = 0; i < (*alignedPCs).size(); ++i) {
        viewer->addPointCloud( (*alignedPCs)[i], "pc"+to_string(i) );
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "pc"+to_string(i));
    }

    // Starting visualizer
    viewer->addCoordinateSystem (0.4, "global");

    // Wait until visualizer window is closed.
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return (0);
}
