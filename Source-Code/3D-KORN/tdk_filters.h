
#ifndef TDK_FILTERS_H
#define TDK_FILTERS_H

#include <QDebug>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/sampling_surface_normal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/processing.h>
#include <pcl/surface/vtk_smoothing/vtk.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkVersion.h>

using namespace pcl;

#include <string>

class TDK_Filters
{
public:
    TDK_Filters();
    ~TDK_Filters();

    //First function for pass through filtering (from 3D Korn)
    static void FilterPCPassthrough(const double &ci, const double &minx,
                                    const double &maxx, const double &miny,
                                    const double &maxy, const double &minz,
                                    const double &maxz, uint &xi, uint &yi, uint &zi,
                                    const pcl::PointCloud<PointXYZ>::Ptr &cloud,
                                    pcl::PointCloud<PointXYZ>::Ptr &cloud_filtered);

    //Second function for pass through filtering
    static void mf_FilterPassthroughBri(const pcl::PointCloud<PointXYZRGB>::Ptr &cloud,
                                        pcl::PointCloud<PointXYZRGB>::Ptr &cloud_filtered,
                                        float x1 = -0.4, float x2 = 0.4, float y1 = -0.85, float y2 = 1.2, float z1 = 0.1, float z2 = 2.0);

    //Function for filtering outliers
    static void mf_FilterStatisticalOutlierRemoval(const pcl::PointCloud<PointXYZ>::Ptr &cloud,
                                                   const pcl::PointCloud<PointXYZ>::Ptr &cloud_filtered, float threshold = 2.5);
    static void mf_FilterStatisticalOutlierRemoval(const pcl::PointCloud<PointXYZRGB>::Ptr &cloud,
                                                   const pcl::PointCloud<PointXYZRGB>::Ptr &cloud_filtered, float threshold = 2.5);

    //Function for downsampling using voxel grid
    static void mf_FilterVoxelGridDownsample(const pcl::PointCloud<PointXYZ>::Ptr &cloud,
                                             pcl::PointCloud<PointXYZ>::Ptr &cloud_filtered, const float &leafsize);
    static void mf_FilterVoxelGridDownsample(const pcl::PointCloud<PointXYZRGB>::Ptr &cloud,
                                             pcl::PointCloud<PointXYZRGB>::Ptr &cloud_filtered, const float &leafsize);

    //Function for smoothing using MLS
    static void mf_FilterMLSSmoothing(const pcl::PointCloud<PointXYZ>::Ptr &cloud,
                                   pcl::PointCloud<PointXYZ>::Ptr &cloud_smoothed, float searchradius);

    //Function for Laplacian Smoothing
    static void mf_FilterLaplacianSmoothing(const boost::shared_ptr<pcl::PolygonMesh> &triangles,
                                                  pcl::PolygonMesh::Ptr &mv_MeshesOutput1);
};

#endif // TDK_FILTERS_H

