#ifndef TDK_MESHING_H
#define TDK_MESHING_H

#include <QDebug>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/marching_cubes_hoppe.h>

#include "tdk_filters.h"

using namespace pcl;

//Class for meshing functions (poisson grid projection and greedy triangulation)
class TDK_Meshing
{
public:
    TDK_Meshing();
    ~TDK_Meshing();

    static void mf_NormalEstimation(PointCloud<PointXYZ>::Ptr &mv_PointCloudInput,
                                          PointCloud<pcl::PointNormal>::Ptr &mv_PointNormalOutput);
    static void mf_ConvertFromXYZRGBtoXYZ(const PointCloud<pcl::PointXYZRGB>::Ptr &mv_PointCloudInput,
                                          PointCloud<pcl::PointXYZ>::Ptr &mv_PointCloudOutput);

    //for input of type PointXYZ
    static void mf_Poisson(const PointCloud<PointXYZ>::Ptr &mv_PointCloudInput, PolygonMesh::Ptr &mv_MeshesOutput);
    static void mf_Poisson(const PointCloud<PointXYZRGB>::Ptr &mv_PointCloudInputRGB,PolygonMesh::Ptr &mv_MeshesOutput);

    static void mf_Greedy_Projection_Triangulation(const PointCloud<PointXYZ>::Ptr &mv_PointCloudInput,
                                       pcl::PolygonMesh::Ptr &mv_MeshesOutput);
    static void mf_Greedy_Projection_Triangulation(const PointCloud<PointXYZRGB>::Ptr &mv_PointCloudInputRGB,
                                       pcl::PolygonMesh::Ptr &mv_MeshesOutput);

    static void mf_Grid_Projection(const PointCloud<PointXYZ>::Ptr &mv_PointCloudInput,
                                       pcl::PolygonMesh::Ptr &mv_MeshesOutput);
    static void mf_Grid_Projection(const PointCloud<PointXYZRGB>::Ptr &mv_PointCloudInputRGB,
                                       pcl::PolygonMesh::Ptr &mv_MeshesOutput);
    
    static void mf_Marching_Cubes(const PointCloud<PointXYZ>::Ptr &mv_PointCloudInput,
                                       pcl::PolygonMesh::Ptr &mv_MeshesOutput);
    static void mf_Marching_Cubes(const PointCloud<PointXYZRGB>::Ptr &mv_PointCloudInputRGB,
                                       pcl::PolygonMesh::Ptr &mv_MeshesOutput);

};

#endif // TDK_MESHING_H
