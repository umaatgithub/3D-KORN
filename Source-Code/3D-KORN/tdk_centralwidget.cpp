#include "tdk_centralwidget.h"

TDK_CentralWidget::TDK_CentralWidget(QWidget *parent) : QWidget(parent),
    layout(new QGridLayout), filtering_axis (1),  // = y
    color_mode(4)  // = Rainbow
{
    cloud.reset(new PointCloudT);
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
    pcl::io::loadPCDFile ("D:\\MAIA\\SEM 1\\Software Engineering\\Project 3D-KORN\\eze_thresholded\\chef.pcd", *cloud);
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].r = 255;
        cloud->points[i].g = 255;
        cloud->points[i].b = 255;
    }

    QDockWidget *dock = new QDockWidget(tr("Scan Widget"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;

    gridLayout->addWidget(new QPushButton("Start scan"), 0, 0);

    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dock->setWidget(scrollArea);
    dock->setFeatures(QDockWidget::NoDockWidgetFeatures);

    layout->addWidget(dock, 0, 0);
    layout->addWidget(new QScrollArea(), 1, 0);
    layout->addWidget(new QScrollArea(), 2, 0);

    QVTKWidget *qvtkWidget = new QVTKWidget;
    layout->addWidget(qvtkWidget, 0, 1, 3, 1);


    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    viewer->setBackgroundColor (0.1, 0.1, 0.1);
    qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor ( qvtkWidget->GetInteractor (), qvtkWidget->GetRenderWindow ());
    //qvtkWidget->update();
    viewer->addPointCloud (cloud, "cloud");
    viewer->resetCamera ();
    qvtkWidget->update ();


//    layout->addWidget(new QScrollArea(), 0, 1, 3, 1);

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
