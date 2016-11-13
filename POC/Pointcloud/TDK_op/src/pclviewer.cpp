#include "pclviewer.h"
#include "../build/ui_pclviewer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>

#include <pcl/io/ply_io.h>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>


#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/impl/passthrough.hpp>
#include "TDK_PointOperations.h"
//sav

#include <iostream>
#include <pcl/common/common.h>

#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/vtk_io.h>
#include <vtkDataReader.h>
#include <vtkGenericDataObjectReader.h>
#include <vtkStructuredGrid.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <string>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("PCL viewer");

  // Setup the cloud pointer
 // cloud.reset (new PointCloudT);
  // The number of points in the cloud

//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

 // pcl::PointCloud<pcl::PointXYZ>::Ptr clouda(new pcl::PointCloud<pcl::PointXYZ>());

  cloud.reset ((new pcl::PointCloud<pcl::PointXYZ>));
  clouda.reset ((new pcl::PointCloud<pcl::PointXYZ>)); //LU : it's a copy where my crop-function acts

  cloud_RGB.reset ((new pcl::PointCloud<pcl::PointXYZRGB>));

  //load a file
  //pcl::io::loadPCDFile<pcl::PointXYZ> ("C:/Users/standard/Desktop/Crapf/chef.pcd" , *input2);

  pcl::io::loadPCDFile<pcl::PointXYZ> ("../chef.pcd" , *cloud);
  pcl::io::loadPCDFile<pcl::PointXYZRGB> ("../chef.pcd" , *cloud_RGB);




  // The default color
  red   = 128;
  green = 128;
  blue  = 128;



  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  // Connect "random" button and the function
  connect (ui->pushButton_random,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));

  // Connect R,G,B sliders and their functions
  connect (ui->horizontalSlider_R, SIGNAL (valueChanged (int)), this, SLOT (redSliderValueChanged (int)));
  connect (ui->horizontalSlider_G, SIGNAL (valueChanged (int)), this, SLOT (greenSliderValueChanged (int)));
  connect (ui->horizontalSlider_B, SIGNAL (valueChanged (int)), this, SLOT (blueSliderValueChanged (int)));
  connect (ui->horizontalSlider_R, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
  connect (ui->horizontalSlider_G, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
  connect (ui->horizontalSlider_B, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));

  // Connect point size slider
  connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

  /*LU: it's a loop to set the minimum and maximum point (coordinates) of the points of the cloud that
   * we have uploaded : it compares every coordinates and search for the min and max
   */




  for (size_t i = 0; i < cloud->points.size (); ++i)
   {

       if (minx > cloud->points[i].x) //check on x

           minx = cloud->points[i].x;

       if (maxx < cloud->points[i].x)
           maxx = cloud->points[i].x;


       if (miny > cloud->points[i].y) //check on y
           miny = cloud->points[i].y;

       if (maxy < cloud->points[i].y)
           maxy = cloud->points[i].y;

       if (minz > cloud->points[i].z) //check on z
           minz = cloud->points[i].z;

       if (maxz < cloud->points[i].z)
           maxz = cloud->points[i].z;

   }
//LU: find the distanze between each direction
  kx=(maxx-minx)/255+minx;

  ky=(maxy-miny)/255+miny;


kz=(maxz-minz)/255+minz;

  viewer->addPointCloud (cloud, "cloud");
  pSliderValueChanged (2);
  viewer->resetCamera ();
  ui->qvtkWidget->update ();
}











void
PCLViewer::randomButtonPressed ()
{
  printf ("Random button was pressed\n");

  // Set the new color
  //pcOperations::FilterPCPassthrough(minx, maxx, miny, maxy, minz, maxz, kx, ky, kz, xi, yi, zi,cloud, clouda);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudcrop ;
  //pcOperations::FilterPCPassthroughNew(input2, cloudcrop);

  pcl::PolygonMesh::Ptr mesh(new pcl:: PolygonMesh());

  TDK_PointOperations::mf_PoissonMeshes(cloud_RGB, mesh);

  viewer->addPolygonMesh(*mesh);
   viewer->resetCamera ();
ui->qvtkWidget->update ();

  viewer->updatePointCloud (cloud_RGB, "cloud");
  ui->qvtkWidget->update ();


}

void
PCLViewer::RGBsliderReleased ()
{
  // Set the new color

  //viewer->updatePointCloud (cloud, "cloud");
  //ui->qvtkWidget->update ();
}


void
PCLViewer::pSliderValueChanged (int value)
{
  conluc = value;
//  CropBoxPointsRemoval(conluc, red, green,blue,0);
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
  //ui->qvtkWidget->update ();
}



void
PCLViewer::redSliderValueChanged (int value)
{
  red = value;
  printf ("redSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
   CropBoxPointsRemoval(conluc, red, green,blue,1);
}

void
PCLViewer::greenSliderValueChanged (int value)
{
  green = value;
  printf ("greenSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
   CropBoxPointsRemoval(conluc, red, green,blue,2);
}


void
PCLViewer::blueSliderValueChanged (int value)
{
  blue = value;
  printf("blueSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
   CropBoxPointsRemoval(conluc, red, green,blue,3);
}


//void PCLViewer::CropBoxPointsRemoval(uint xi, uint yi, uint zi,uint axe){

void PCLViewer::CropBoxPointsRemoval(uint ci, uint xi, uint yi, uint zi,uint axe){




 //viewer->addPolygonMesh(mesh);
  viewer->resetCamera ();
ui->qvtkWidget->update ();



}

PCLViewer::~PCLViewer ()
{
  delete ui;
}
