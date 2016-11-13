#ifndef PCOPERATIONS_H
#define PCOPERATIONS_H

#include "pclviewer.h"
#include "../build/ui_pclviewer.h"
#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>


using namespace pcl;

class TDK_PointOperations
{
public:
    TDK_PointOperations();

    //function for trying class -> i dont use it right now
    static void FilterPCPassthrough(const double &minx, const double &maxx, const double &miny, const double &maxy, const double &minz, const double &maxz, const double &kx, const double &ky, const double &kz, uint &xi, uint &yi, uint &zi, const PointCloud<PointXYZ>::Ptr &input, PointCloud<PointXYZ>::Ptr &output);

    //new passthrough filter
    static void mf_FilterPassthrough(const PointCloud<PointXYZ>::Ptr &PointCloudInput, PointCloud<PointXYZ>::Ptr &PointCloudOutput);
    static void mf_NormalEstimation(PointCloud<PointXYZ>::Ptr &PointCloudInput, PointCloud<pcl::PointNormal>::Ptr &PointNormalOutput);

    //for input of type PointXYZRGB
    static void mf_PoissonMeshes(const PointCloud<PointXYZRGB>::Ptr &PointCloudInput , PolygonMesh::Ptr &MeshesOutput);

    //for input of type PointXYZ
    static void mf_PoissonMeshes(const PointCloud<PointXYZ>::Ptr &PointCloudInput , PolygonMesh::Ptr &MeshesOutput);
    static void mf_ConvertFromXYZRGBtoXYZ(const PointCloud<pcl::PointXYZRGB>::Ptr &PointCloudInput, PointCloud<pcl::PointXYZ>::Ptr &PointCloudOutput);

};

#endif // TDK_POINTOPERATIONS_H
