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
    int numPointclouds = 10;

    TDK_ScanRegistration scanRegistrator;

    if(argc == 4 && false){
        scanRegistrator.setMv_ISS_resolution((float)atof(argv[1]));
        scanRegistrator.setMv_SVD_MaxDistance((float)atof(argv[2]));
        scanRegistrator.setMv_ICP_MaxCorrespondenceDistance((float)atof(argv[3]));

        for (int i = 0; i < argc; ++i) {
            qDebug() << (float)atof(argv[i]);
        }
    }

    //Load pc
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    for (int i = 1; i <= numPointclouds; ++i) {
        cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPLYFile("../alb" + to_string(i) +".ply", *cloud);
        //add to myRegistrator
        scanRegistrator.addNextPointCloud(cloud);
        qDebug() << "Number of keypoints = " << scanRegistrator.getLastDownSampledPointcloud()->points.size();

        //qDebug() << "../alb" << QString::fromStdString(to_string(i)) << ".ply " << " anyadido y procesado";
    }


    // Initializing point cloud visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
            viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

//    viewer->addPointCloud( cloud, "pc2");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "pc2");


    //    viewer->addPointCloud( scanRegistrator.getLastDownSampledPointcloud(), "pc1");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "pc1");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "pc1");
    //add to viewer
    vector<TDK_ScanRegistration::PointCloudT::Ptr> * alignedPCs = scanRegistrator.mf_getAlignedPointClouds();


    for (int i = 0; i < (*alignedPCs).size(); ++i) {
        viewer->addPointCloud( (*alignedPCs)[i], "pc"+to_string(i) );
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "pc"+to_string(i));

        //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0.8, "pc2");
    }

    // Starting visualizer
    //viewer->addCoordinateSystem (1.0, "global");

    // Wait until visualizer window is closed.
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return (0);
}
