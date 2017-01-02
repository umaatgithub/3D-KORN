#include "TDK_LoadSaveFile.h"
#include "vtkPolyDataReader.h"
#include "vtkSTLWriter.h"
#include "vtkTriangleFilter.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "pclviewer.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <QFileDialog>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>

TDK_LoadSaveFile::TDK_LoadSaveFile()
{

}

bool TDK_LoadSaveFile::VTK2STL(const char * vtkFile, const char * stlFile)
{
    vtkPolyDataReader *reader=vtkPolyDataReader::New();
    reader->SetFileName(vtkFile);
    vtkSTLWriter *writer=vtkSTLWriter::New();

    vtkTriangleFilter *tri=vtkTriangleFilter::New();
    tri->SetInput(reader->GetOutput());


    writer->SetInputConnection(tri->GetOutputPort());
    writer->SetFileName(stlFile);
    writer->Update();
    writer->Write();

    return true;
}

bool TDK_LoadSaveFile::LoadPCD(const char * pcdFile, pcl::PointCloud<pcl::PointXYZ>::Ptr &loaded_cloud)
{

    pcl::io::loadPCDFile(pcdFile, *loaded_cloud);

    return true;

}

bool TDK_LoadSaveFile::LoadPLY(const char * plyFile, pcl::PointCloud<pcl::PointXYZ>::Ptr &loaded_cloud )
{
     pcl::io::loadPLYFile(plyFile, *loaded_cloud);

     return true;

}


bool TDK_LoadSaveFile::SavePCD(const char * pcdFile, pcl::PointCloud<pcl::PointXYZ>::Ptr &saved_cloud )
{

    pcl::io::savePCDFile(pcdFile, *saved_cloud);
    //pcl::io::savePCDFileASCII(pcdFile, saved_cloud);  //the same

    return true;
}

bool TDK_LoadSaveFile::SaveVTK(const char * vtkFile, pcl::PolygonMesh::Ptr &mesh)
{
    pcl::io::saveVTKFile(vtkFile, *mesh);

    return true;
}

bool TDK_LoadSaveFile::LoadVTK(const char * vtkFile, pcl::PolygonMesh::Ptr &mesh)
{
    pcl::io::loadPolygonFileVTK(vtkFile, *mesh);

    return true;

}

bool TDK_LoadSaveFile::SaveSTL(const char * stlFile, const char * vtkFile, pcl::PolygonMesh::Ptr &mesh)
{
    LoadVTK(vtkFile, mesh);
    VTK2STL(vtkFile, stlFile);

    return true;

}
