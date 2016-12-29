#ifndef TDK_LOADSAVEFILE_H
#define TDK_LOADSAVEFILE_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "pclviewer.h"

class TDK_LoadSaveFile
{
    pcl::PointCloud<pcl::PointXYZ> *load_cloud;
    pcl::PointCloud<pcl::PointXYZ> *save_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PolygonMesh::Ptr *mesh;


public:
    TDK_LoadSaveFile();
    static void VTK2STL(const char *, const char *);
    static void LoadPCD(const char *, pcl::PointCloud<pcl::PointXYZ>::Ptr &  );
    static void SavePCD(const char *, pcl::PointCloud<pcl::PointXYZ>::Ptr & );
    static void SaveVTK(const char *, pcl::PolygonMesh::Ptr &);
    static void LoadVTK(const char *, pcl::PolygonMesh::Ptr &);
    static void SaveSTL(const char *, const char * , pcl::PolygonMesh::Ptr &);

};

#endif // TDK_LOADSAVEFILE_H
