#include "tdk_centralwidget.h"

TDK_CentralWidget::TDK_CentralWidget(QWidget *parent) : QWidget(parent),
    layout(new QGridLayout), filtering_axis (1),  // = y
    color_mode(4)  // = Rainbow
{
    vtkObject::GlobalWarningDisplayOff();
//    cloudSource1.reset(new PointCloudT);
//    cloudSource2.reset(new PointCloudT);
 //   cloudRegistered.reset(new PointCloudT);
      // The number of points in the cloud
//      cloud->resize (500);

      // Fill the cloud with random points
//      for (size_t i = 0; i < cloud->points.size (); ++i)
//      {
//        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
//        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
//        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);

//        cloud->points[i].r = 0;
//        cloud->points[i].g = 255;
//        cloud->points[i].b = 0;
//      }

    //PointCloudT::Ptr cloud_tmp (new PointCloudT);
//    pcl::io::loadPCDFile ("D:\\MAIA\\SEM 1\\Software Engineering\\Project 3D-KORN\\eze_thresholded\\test_pcd_0.pcd", *cloud);
//    pcl::io::loadPCDFile ("D:\\MAIA\\SEM 1\\Software Engineering\\Project 3D-KORN\\eze_thresholded\\chef.pcd", *cloud);
//    pcl::io::loadPLYFile ("D:\\MAIA\\SEM 1\\Software Engineering\\Project 3D-KORN\\eze_thresholded\\eze_3.ply", *cloudSource1);
//    pcl::io::loadPLYFile ("D:\\MAIA\\SEM 1\\Software Engineering\\Project 3D-KORN\\eze_thresholded\\eze_1.ply", *cloudSource2);

//    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
//    // Set the input source and target
//    icp.setInputCloud (cloudSource1);
//    icp.setInputTarget (cloudSource2);
//    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
//    icp.setMaxCorrespondenceDistance (0.05);
//    // Set the maximum number of iterations (criterion 1)
//    icp.setMaximumIterations (50);
//    // Set the transformation epsilon (criterion 2)
//    icp.setTransformationEpsilon (1e-8);
//    // Set the euclidean distance difference epsilon (criterion 3)
//    icp.setEuclideanFitnessEpsilon (1);
//    // Perform the alignment
//    icp.align(*cloudRegistered);
//    // Obtain the transformation that aligned cloud_source to cloud_source_registered
//    Eigen::Matrix4f transformation = icp.getFinalTransformation ();

//    for (size_t i = 0; i < cloud->points.size (); ++i)
//    {
//        cloud->points[i].r = 255;
//        cloud->points[i].g = 255;
//        cloud->points[i].b = 255;
//    }

    QDockWidget *dock = new QDockWidget(tr("Dummy"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;

//    gridLayout->addWidget(new QPushButton("Start scan"), 0, 0);

    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dock->setWidget(scrollArea);
    dock->setFeatures(QDockWidget::NoDockWidgetFeatures);

    layout->addWidget(dock, 0, 0);
    layout->addWidget(new QScrollArea(), 1, 0);
    layout->addWidget(new QScrollArea(), 2, 0);

//    QVTKWidget *qvtkWidget = new QVTKWidget;
//    layout->addWidget(qvtkWidget, 0, 1, 3, 1);


//    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
//    viewer->setBackgroundColor (0.1, 0.1, 0.1);
//    qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
//    viewer->setupInteractor ( qvtkWidget->GetInteractor (), qvtkWidget->GetRenderWindow ());
//    viewer->addPointCloud (cloudSource1, "cloud");
//    viewer->resetCamera ();
//    qvtkWidget->update ();


    layout->addWidget(new QScrollArea(), 0, 1, 3, 1);

    layout->addWidget(new QScrollArea(), 0, 2);
    layout->addWidget(new QScrollArea(), 1, 2);
    layout->addWidget(new QScrollArea(), 2, 2);

    layout->setColumnMinimumWidth(0, 300);
    layout->setColumnMinimumWidth(1, 300);
    layout->setColumnMinimumWidth(2, 300);

    layout->setRowMinimumHeight(0, 150);
    layout->setRowMinimumHeight(1, 150);
    layout->setRowMinimumHeight(2, 150);

    layout->setColumnStretch(1, 1);

//    layout->setRowStretch(0, 1);
//    layout->setRowStretch(1, 1);
//    layout->setRowStretch(2, 1);

//    layout->setMargin(10);

    setLayout(layout);
}

TDK_CentralWidget::~TDK_CentralWidget()
{

}
