// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
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
  TDK_ScanRegistration myRegistrator;
  TDK_ScanRegistration myRegistrator2(165);

  // Initializing point cloud visualizer
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return (0);
}
