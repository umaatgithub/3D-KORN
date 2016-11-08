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
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZRGBA PointType;
const float bad_point = std::numeric_limits<float>::quiet_NaN();

int
main (int argc, char** argv)
{
  TDK_ScanRegistration myRegistrator;

  //Load pc
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::io::loadPCDFile("../alb1.pcd", *cloud);

  //add to myRegistrator
  myRegistrator.addNextPointCloud(cloud);

  //get from myRegsitrator
  cloud = myRegistrator.getLastOriginalPointcloud();



  // Initializing point cloud visualizer
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  //add to viewer
  viewer->addPointCloud( cloud, "final2" );
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "final2");

  // Starting visualizer
  viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return (0);
}
