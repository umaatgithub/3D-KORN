// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>

#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>


#include <QDebug>
#include <QString>


typedef pcl::PointXYZRGB PointType;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

class FeatureCloudContainer
{
  public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZRGB> SearchMethod;

    FeatureCloudContainer () :
      search_method_xyz_ (new pcl::search::KdTree<pcl::PointXYZRGB>),
      normal_radius_ (0.05f),
      feature_radius_ (0.05f)
    {}

    ~FeatureCloudContainer () {}

    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      pcl::io::loadPCDFile (pcd_file, *xyz_);
      processInput ();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
      qDebug() << "Computing Normals";
      computeSurfaceNormals ();
      qDebug() << "Computing Features";
      computeLocalFeatures ();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

int main( int argc, char* argv[] )
{
    // Create a boost pointer to a PCLVisualizer window
    // (boost is for something about concurrency and sharing pointers through the code)
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
            new pcl::visualization::PCLVisualizer( "Point Cloud Viewer") );
    viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0, 0.0, 0.0 );

    //Create two cloud pointers and initialize them (if we dont initialize they will point to NULL)
    pcl::PointCloud<PointType>::Ptr cloud1Original(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud2Original(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud1(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud2(new pcl::PointCloud<PointType>);

    //Load the two stored .ply files into cloud1 and cloud 2
    pcl::io::loadPLYFile("../../../models/eze_3.ply", *cloud1Original);
    pcl::io::loadPLYFile("../../../models/eze_4.ply", *cloud2Original);

    qDebug() << "Cloud1 has " << static_cast<int>(cloud1Original->points.size()) << " points";
    qDebug() << "Cloud2 has " << static_cast<int>(cloud2Original->points.size()) << " points";

    //Filter out 0,0,0 points
    const float depth_limit = 6.0;
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.00001, depth_limit);

    pass.setInputCloud (cloud1Original);
    pass.filter (*cloud1Original);
    pass.setInputCloud (cloud2Original);
    pass.filter (*cloud2Original);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    float leafSize = 0.02f;
    sor.setLeafSize (leafSize, leafSize, leafSize);

    sor.setInputCloud (cloud1Original);
    sor.filter (*cloud1);
    sor.setInputCloud (cloud2Original);
    sor.filter (*cloud2);

    qDebug() << "Filtered Cloud1 has " << static_cast<int>(cloud1->points.size()) << " points";
    qDebug() << "Filtered Cloud2 has " << static_cast<int>(cloud2->points.size()) << " points";

    //Inform user that reading has been succesfull :D
    qDebug() << "Read PlY";

    //---------------- Features -------------------------
    FeatureCloudContainer source, target;
    source.setInputCloud(cloud1);
    target.setInputCloud(cloud2);

    qDebug() << "Finish compute features";
    //-------------------------- SAC -------------------------------
    //Create initial SAC aligner
    pcl::SampleConsensusInitialAlignment<PointType, PointType, pcl::FPFHSignature33> sac;

    //Configure basic SAC parameters
    float min_sample_distance (0.01f);
    float max_correspondence_distance (0.1f);
    int nr_iterations (500);
    sac.setMinSampleDistance (min_sample_distance);
    sac.setMaxCorrespondenceDistance (max_correspondence_distance);
    sac.setMaximumIterations (nr_iterations);

    qDebug() << "Set Cloud1";
    //Set input cloud to be transformed
    sac.setInputCloud (cloud1);
    sac.setSourceFeatures (source.getLocalFeatures());

    qDebug() << "Set Cloud2";
    //Set target cloud of reference
    sac.setInputTarget (cloud2);
    sac.setTargetFeatures (target.getLocalFeatures());

    qDebug() << "Start SAC";

    //Perform rough alignment
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_source(new pcl::PointCloud<PointType>);
    sac.align (* aligned_source);
    Eigen::Matrix4f initial_T = sac.getFinalTransformation ();

    qDebug() << "Finished SAC with score " << sac.getFitnessScore();

    cloud1 = aligned_source;
    pcl::transformPointCloud(*cloud1Original, *cloud1Original, initial_T);


    //------------------- ICP ---------------------------------

    //Create the ICP "tool" object
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    //Set cloud1 as transformable cloud for icp algorithm
    icp.setInputCloud(cloud1);
    //Set cloud2 as target cloud for icp algorithm
    icp.setInputTarget(cloud2);
    qDebug() << "Start ICP";
    //Execute the algorithm and output result to "final", that will now hold the transformed cloud
    pcl::PointCloud<pcl::PointXYZRGB> aligned_source2;
    icp.align(aligned_source2);

    pcl::transformPointCloud(*cloud1Original, *cloud1Original, icp.getFinalTransformation());

    // Check if algorithm has converged and alignment has been succesful
    if(!icp.hasConverged()) return 0;
    qDebug() << "ICP converged! with score " << icp.getFitnessScore();

    qDebug() << "Joining Pointclouds";

    pcl::PointCloud<PointType>::Ptr final;
    final = cloud1Original;
    cloud2 = cloud2Original;

    //Join both pointclouds in final2 for display
    int combinedSize = final->points.size() + cloud2->points.size();
    pcl::PointCloud<PointType> final2;

    //Add points from final into final2
    for (int i = 0; i < final->points.size(); ++i) {
        final2.points.push_back( final->points[i]);
    }
    //Add points from cloud2 into final2
    for (int i = final->points.size(); i < combinedSize; ++i) {
        final2.points.push_back((*cloud2.get()).points[i-final->points.size()]);
    }

    qDebug() << final2.points.size();
    qDebug() << "Finished ICP";

    //pcl::io::savePLYFileBinary ("icp.ply" , final);
    qDebug() << "Showing";

   viewer->spinOnce();

   viewer->addCoordinateSystem();
   viewer->addPointCloud( final2.makeShared(), "final2" );
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "final2");

   while (!viewer->wasStopped ())
   {
     viewer->spinOnce (100);
     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   }

   return 0;
}

/*
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

class FeatureCloud
{
  public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.02f),
      feature_radius_ (0.02f)
    {}

    ~FeatureCloud () {}

    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      pcl::io::loadPCDFile (pcd_file, *xyz_);
      processInput ();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment
{
  public:

    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      Eigen::Matrix4f final_transformation;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment () :
      min_sample_distance_ (0.05f),
      max_correspondence_distance_ (0.01f*0.01f),
      nr_iterations_ (500)
    {
      // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
      sac_ia_.setMinSampleDistance (min_sample_distance_);
      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
      sac_ia_.setMaximumIterations (nr_iterations_);
    }

    ~TemplateAlignment () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
      target_ = target_cloud;
      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
    {
      templates_.push_back (template_cloud);
    }

    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
      sac_ia_.setInputCloud (template_cloud.getPointCloud ());
      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

      pcl::PointCloud<pcl::PointXYZ> registration_output;
      sac_ia_.align (registration_output);

      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
      result.final_transformation = sac_ia_.getFinalTransformation ();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
      results.resize (templates_.size ());
      for (size_t i = 0; i < templates_.size (); ++i)
      {
        align (templates_[i], results[i]);
      }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result)
    {
      // Align all of the templates to the target cloud
      std::vector<Result, Eigen::aligned_allocator<Result> > results;
      alignAll (results);

      // Find the template with the best (lowest) fitness score
      float lowest_score = std::numeric_limits<float>::infinity ();
      int best_template = 0;
      for (size_t i = 0; i < results.size (); ++i)
      {
        const Result &r = results[i];
        if (r.fitness_score < lowest_score)
        {
          lowest_score = r.fitness_score;
          best_template = (int) i;
        }
      }

      // Output the best alignment
      result = results[best_template];
      return (best_template);
    }

  private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};

// Align a collection of object templates to a sample point cloud
int
main (int argc, char **argv)
{
  if (argc < 3)
  {
    printf ("No target PCD file given!\n");
    return (-1);
  }

  // Load the object templates specified in the object_templates.txt file
  std::vector<FeatureCloud> object_templates;
  std::ifstream input_stream (argv[1]);
  object_templates.resize (0);
  std::string pcd_filename;
  while (input_stream.good ())
  {
    std::getline (input_stream, pcd_filename);
    if (pcd_filename.empty () || pcd_filename.at (0) == '#') // Skip blank lines or comments
      continue;

    FeatureCloud template_cloud;
    template_cloud.loadInputCloud (pcd_filename);
    object_templates.push_back (template_cloud);
  }
  input_stream.close ();

  // Load the target cloud PCD file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (argv[2], *cloud);

  // Preprocess the cloud by...
  // ...removing distant points
  const float depth_limit = 1.0;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, depth_limit);
  pass.filter (*cloud);

  // ... and downsampling the point cloud
  const float voxel_grid_size = 0.005f;
  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
  vox_grid.setInputCloud (cloud);
  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
  vox_grid.filter (*tempCloud);
  cloud = tempCloud;

  // Assign to the target FeatureCloud
  FeatureCloud target_cloud;
  target_cloud.setInputCloud (cloud);

  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  for (size_t i = 0; i < object_templates.size (); ++i)
  {
    template_align.addTemplateCloud (object_templates[i]);
  }
  template_align.setTargetCloud (target_cloud);

  // Find the best template alignment
  TemplateAlignment::Result best_alignment;
  int best_index = template_align.findBestAlignment (best_alignment);
  const FeatureCloud &best_template = object_templates[best_index];

  // Print the alignment fitness score (values less than 0.00002 are good)
  printf ("Best fitness score: %f\n", best_alignment.fitness_score);

  // Print the rotation matrix and translation vector
  Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
  Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);

  printf ("\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
  printf ("\n");
  printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

  // Save the aligned template for visualization
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  pcl::transformPointCloud (*best_template.getPointCloud (), transformed_cloud, best_alignment.final_transformation);
  pcl::io::savePCDFileBinary ("output.pcd", transformed_cloud);

  return (0);
}
*/
