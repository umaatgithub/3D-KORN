#ifndef TDK_LOADSAVEFILE_H
#define TDK_LOADSAVEFILE_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "pclviewer.h"

class TDK_LoadSaveFile
{


public:
    TDK_LoadSaveFile();
    static bool VTK2STL(const char * vtkFile, const char * stlFile);
    static bool LoadPCD(const char * pcdFile, pcl::PointCloud<pcl::PointXYZ>::Ptr & loaded_cloud);
    static bool LoadPLY(const char * plyFile, pcl::PointCloud<pcl::PointXYZ>::Ptr & loaded_cloud );
    static bool SavePCD(const char * pcdFile, pcl::PointCloud<pcl::PointXYZ>::Ptr & saved_cloud);
    static bool SaveVTK(const char * vtkFile, pcl::PolygonMesh::Ptr & mesh);
    static bool LoadVTK(const char * vtkFile, pcl::PolygonMesh::Ptr & mesh);
    static bool SaveSTL(const char * stlFile, const char * vtkFile, pcl::PolygonMesh::Ptr & mesh);

    // The "..File" is the path ended by corresponding file type, for example : vtkFile = "../xxx.vtk"


};

#endif // TDK_LOADSAVEFILE_H
