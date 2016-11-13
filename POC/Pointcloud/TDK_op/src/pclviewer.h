#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream>

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PointIndices.h>

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
    //I put public minx.. and cropbox
    double minx,maxx,miny,maxy,minz,maxz,kx,ky,kz;
    void CropBoxPointsRemoval(uint ci, uint xi, uint yi, uint zi, uint axe);

public slots:
  void
  randomButtonPressed ();

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


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr input2 ;

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB ;


  pcl::PointCloud<pcl::PointXYZ>::Ptr clouda ;
  pcl::PointIndices::Ptr cloudax ; //new std::vector <int>;
  pcl::PointIndices::Ptr clouday ; //(std::vector <int>);
  //pcl::PointCloud<pcl::PointIndices> clouday;
  pcl::PointIndices cloudaz (std::vector <int>);

/*
  double minx,maxx,miny,maxy,minz,maxz,kx,ky,kz;
  //PolygonMesh mesh;

  void CropBoxPointsRemoval(uint ci, uint xi, uint yi, uint zi, uint axe);
  */
  unsigned int conluc;
  unsigned int red;
  unsigned int green;
  unsigned int blue;

private:
  Ui::PCLViewer *ui;

};

#endif // PCLVIEWER_H
