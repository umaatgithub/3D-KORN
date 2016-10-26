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

typedef pcl::PointXYZRGBA PointType;
const float bad_point = std::numeric_limits<float>::quiet_NaN();

int
main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr temporal_input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // Loading first scan of room.
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPLYFile("../eze_2.ply", *temporal_input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file eze_2.ply \n");
    return (-1);
  }

  //Eliminate null points
  int numberGoodPoints = 0;
  for (size_t i = 0; i < temporal_input_cloud->points.size (); ++i){
     if(temporal_input_cloud->points[i].x != 0 || temporal_input_cloud->points[i].y != 0
             || temporal_input_cloud->points[i].z != 0){ //1 sweet spot
         numberGoodPoints++;
         target_cloud->resize(numberGoodPoints);
         target_cloud->points[numberGoodPoints-1] = temporal_input_cloud->points[i];
     }
  }


  std::cout << "Loaded " << target_cloud->size () << " data points from eze_2.ply" << std::endl;

  // Loading second scan of room from new perspective.
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPLYFile("../eze_3.ply", *temporal_input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file eze_3.ply \n");
    return (-1);
  }

  //Eliminate null points
  numberGoodPoints = 0;
  for (size_t i = 0; i < temporal_input_cloud->points.size (); ++i){
     if(temporal_input_cloud->points[i].x != 0 || temporal_input_cloud->points[i].y != 0
             || temporal_input_cloud->points[i].z != 0){ //1 sweet spot
         numberGoodPoints++;
         input_cloud->resize(numberGoodPoints);
         input_cloud->points[numberGoodPoints-1] = temporal_input_cloud->points[i];
     }
  }

  std::cout << "Loaded " << input_cloud->size () << " data points from eze_3.ply" << std::endl;

  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;

  //LeafSize should assure 6points per voxel, but not many more than that :O
  float leafSize = 0.025;
  approximate_voxel_filter.setLeafSize (leafSize, leafSize, leafSize);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
            << " data points from eze_3.pcd" << std::endl;

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.01); //orig 0.01
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.01); //orig 0.1
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (0.01); //orig 1.0

  // Setting max number of registration iterations.
  ndt.setMaximumIterations (100); //orig 35

  // Setting point cloud to be aligned.
  ndt.setInputSource (filtered_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (target_cloud);

  // Set initial alignment estimate found using robot odometry.
  Eigen::AngleAxisf init_rotation (1.5, Eigen::Vector3f::UnitY ());
  Eigen::Translation3f init_translation (0.02, -0.01, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align (*output_cloud, init_guess);

  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());

  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

  // Initializing point cloud visualizer
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");

  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  output_color (output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");

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
