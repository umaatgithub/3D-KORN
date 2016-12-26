#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream>

// Qt
#include <QMainWindow>

// Point Cloud Library

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/vtk_io.h>

using namespace pcl;
using namespace std;

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCLViewer (QWidget *parent = 0);
  ~PCLViewer ();

    void RandomPlane(int numberofpoints,int randprecision,uint xvalue, uint yvalue,uint zvalue,uint axe);
    void CropPlanesPointsRemoval(uint xi, uint yi, uint zi,uint axe);


    double minx,maxx,miny,maxy,minz,maxz,kx,ky,kz;

public slots:
  void
  randomButtonPressed ();

  void load();

  void
  RGBsliderReleased ();

  void
  pSliderValueChanged (int value);

  void
  redSliderValueChanged (int value);

  void
  greenSliderValueChanged (int value);

  void
  blueSliderValueChanged (int value);

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
 // PointCloudT::Ptr cloud;
  PointCloud<PointXYZ>::Ptr cloud ;

  PointCloud<PointXYZ>::Ptr cloud_hullx ;

  PointCloud<PointXYZ>::Ptr cloud_hully ;


  PointCloud<PointXYZ>::Ptr cloud_hullz ;


  PointCloud<PointXYZ>::Ptr cloud_randomplane ;


  PointCloud<PointXYZ>::Ptr objects ;


  PointCloud<PointXYZRGBA>::Ptr potatoecloud ;


  PolygonMesh mesh;
std::vector<std::pair<float, float> > outliercoordinates;

  void  FillingCropPlane( );
  PointCloud<PointXYZ>::Ptr cloud_cropped ;


double croppedcloudsize;
  unsigned int red;
  unsigned int green;
  unsigned int blue;

private slots:
  void on_load_clicked();

private:
  Ui::PCLViewer *ui;

};

#endif // PCLVIEWER_H
