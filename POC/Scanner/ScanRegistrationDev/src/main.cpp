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

typedef pcl::PointXYZRGB PointType;

double
computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (size_t i = 0; i < cloud->size (); ++i)
  {
    if (! pcl_isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

int
main (int argc, char** argv)
{
    int numPointclouds = 8;

    //pcl::PointXYZ scannerCenter((float)atof(argv[1]), (float)atof(argv[2]), (float)atof(argv[3]));
    TDK_ScanRegistration scanRegistrator;

    // x center = 0.087, z_center = 1.4
    pcl::PointXYZ scannerCenter(0.08, 0.0, 1.43); //2.054 obtained from max_z in center slice
    scanRegistrator.setMv_scannerCenterRotation(scannerCenter);

    //Load pc
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    for (int i = 0; i < numPointclouds; ++i) {
        cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPLYFile("../chair_eze" + to_string(i) +".ply", *cloud);

        scanRegistrator.setMv_ISS_resolution(computeCloudResolution(cloud));

        //add to myRegistrator
        scanRegistrator.addNextPointCloud(cloud, -35.0); //33

        /*
        //Estimate center of rot
        float min_z = 10000;
        float max_z = 0;
        float mean_x = 0;
        int num_x = 0;

        for (int j = 0; j < cloud->points.size(); ++j) {
           if( cloud->points[j].y < 0.2 && cloud->points[j].y > 0.1){
               if(min_z > cloud->points[j].z){
                   min_z = cloud->points[j].z;
               }

               if(max_z < cloud->points[j].z){
                   max_z = cloud->points[j].z;
               }

               mean_x += cloud->points[j].x;
               num_x++;
           }
        }

        mean_x /= num_x;

        qDebug() << "Min Z: " << min_z;
        qDebug() << "Max Z: " << max_z;
        qDebug() << "Mean X: " << mean_x;
        */

        qDebug() << "Number of keypoints = " << scanRegistrator.getLastDownSampledPointcloud()->points.size();
    }


    // Initializing point cloud visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
            viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    //add to viewer
    //vector<TDK_ScanRegistration::PointCloudT::Ptr> * alignedPCs = scanRegistrator.mf_getOriginalPointClouds();
    vector<TDK_ScanRegistration::PointCloudT::Ptr> * alignedPCs = scanRegistrator.mf_getAlignedPointClouds();

    for (int i = 0; i < (*alignedPCs).size(); ++i) {
        viewer->addPointCloud( (*alignedPCs)[i], "pc"+to_string(i) );
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "pc"+to_string(i));
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
