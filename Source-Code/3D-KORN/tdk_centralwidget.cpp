#include "tdk_centralwidget.h"
#include <QDebug>
#include <pcl/registration/incremental_registration.h>
#include <QPushButton>
#include <QProgressDialog>
#include <QComboBox>
#include <QCheckBox>


TDK_CentralWidget::TDK_CentralWidget(QWidget *parent) : QWidget(parent),
    layout(new QGridLayout), filtering_axis (1),  // = y
    color_mode(4) ,
    qvtkWidget(new QVTKWidget)
{
    resize(300, 400);

    Setup_CroppingWidget();
    Setup_RegistrationWidget();
    Setup_ProgressionBarWidget();
    Setup_PCListWidget();
    Setup_InformationPCWidget();



    layout->setColumnMinimumWidth(0, 300);
    layout->setColumnMinimumWidth(1, 300);
    layout->setColumnMinimumWidth(2, 300);

    layout->setRowMinimumHeight(0, 300);
    layout->setRowMinimumHeight(1, 300);
    layout->setRowMinimumHeight(2, 300);

    layout->setColumnStretch(1, 1);

    setLayout(layout);


}

void TDK_CentralWidget::Setup_CroppingWidget()
{

    QDockWidget *dockWidget = new QDockWidget(tr("Cropping"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;
    widget->setLayout(gridLayout);



    QPushButton *pb= new QPushButton("Merging" ,widget);

    // Determine in which cell of the grid the QPush will be

    gridLayout->addWidget(pb, 1, 0);

    // Set the dimensions of the grid -> Position in the layout (By nbr of cell)

    gridLayout->setColumnMinimumWidth(0, 150);
    gridLayout->setRowMinimumHeight(0, 100);



    QComboBox *CBox = new QComboBox(widget);
    CBox->addItem("PC 1");
    CBox->addItem("PC 2");
    CBox->addItem("PC 3");

    gridLayout->addWidget(CBox,1, 1);


    QComboBox *CBox1 = new QComboBox(widget);
    CBox1->addItem("PC 1");
    CBox1->addItem("PC 2");
    CBox1->addItem("PC 3");
    CBox1->removeItem(1);

    gridLayout->addWidget(CBox1, 2, 1);







    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    layout->addWidget(dockWidget, 0, 0);
}

void TDK_CentralWidget::Setup_RegistrationWidget()
{

    QDockWidget *dockWidget = new QDockWidget(tr("Registration"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;

    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    layout->addWidget(dockWidget, 1, 0);

}

void TDK_CentralWidget::Setup_ProgressionBarWidget()
{

    QDockWidget *dockWidget = new QDockWidget(tr("Progression"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;



    QProgressDialog *Bar = new QProgressDialog();

    Bar->setLabelText("Position6");
    Bar->setValue(6);
    Bar->setMinimum(0);
    Bar->setMaximum(10);
    gridLayout->addWidget(Bar, 0, 1);



    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    layout->addWidget(dockWidget, 2, 0);
}

void TDK_CentralWidget::Setup_PCListWidget()
{

    QDockWidget *dockWidget = new QDockWidget(tr("Point Cloud List"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;

    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    layout->addWidget(dockWidget, 0, 2);

}

void TDK_CentralWidget::Setup_InformationPCWidget()
{

    QDockWidget *dockWidget = new QDockWidget(tr("Point Cloud Information"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;

    widget->setLayout(gridLayout);



    QCheckBox *checkBox1 = new QCheckBox(tr("&Checkbox 1"));
    QCheckBox *checkBox2 = new QCheckBox(tr("C&heckbox 2"));

    // Set if the button can be checked
    checkBox1->setCheckable(false);

    gridLayout->addWidget(checkBox1, 0, 0);
    gridLayout->addWidget(checkBox2, 1, 0);



    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    layout->addWidget(dockWidget, 1, 2);

}



//TDK_CentralWidget::TDK_CentralWidget(QWidget *parent) : QWidget(parent),
//    layout(new QGridLayout), filtering_axis (1),  // = y
//    color_mode(4) , // = Rainbow
//    qvtkWidget(new QVTKWidget)
//{
//    vtkObject::GlobalWarningDisplayOff();

//    QDockWidget *dock = new QDockWidget(tr("Dummy"));
//    QGridLayout *gridLayout = new QGridLayout;
//    QScrollArea *scrollArea = new QScrollArea;
//    QWidget *widget = new QWidget;


//    widget->setLayout(gridLayout);
//    scrollArea->setWidget(widget);
//    dock->setWidget(scrollArea);
//    dock->setFeatures(QDockWidget::NoDockWidgetFeatures);

//    layout->addWidget(dock, 0, 0);
//    layout->addWidget(new QScrollArea(), 1, 0);
//    layout->addWidget(new QScrollArea(), 2, 0);

//    QDockWidget *dockWidget = new QDockWidget(tr("Sensor Output"));
//    dockWidget->setWidget(qvtkWidget);
//    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
//    layout->addWidget(dockWidget, 0, 1,3, 1);

//    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
//    viewer->setBackgroundColor (0.1, 0.1, 0.1);
//    qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
//    viewer->setupInteractor ( qvtkWidget->GetInteractor (), qvtkWidget->GetRenderWindow ());
//    viewer->resetCamera ();


//    layout->addWidget(new QScrollArea(), 0, 2);
//    layout->addWidget(new QScrollArea(), 1, 2);
//    layout->addWidget(new QScrollArea(), 2, 2);

//    layout->setColumnMinimumWidth(0, 300);
//    layout->setColumnMinimumWidth(1, 300);
//    layout->setColumnMinimumWidth(2, 300);

//    layout->setRowMinimumHeight(0, 150);
//    layout->setRowMinimumHeight(1, 150);
//    layout->setRowMinimumHeight(2, 150);

//    layout->setColumnStretch(1, 1);

//    setLayout(layout);




//    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
//    //pcl::io::loadPLYFile ("C:\\Users\\Umamaheswaran\\Desktop\\testdata\\chair_alb0.ply", *cloud);
//    pcl::io::loadPLYFile ("D:\\MAIA\\SEM 1\\Software Engineering\\Project 3D-KORN\\eze_thresholded\\eze_0.ply", *cloud);
//    TDK_Database::mv_PointCloudsVector.push_back(cloud->makeShared());
////    viewer->addPointCloud( cloud, "cloud1" );
////    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");

//    pcl::io::loadPLYFile ("D:\\MAIA\\SEM 1\\Software Engineering\\Project 3D-KORN\\eze_thresholded\\eze_1.ply", *cloud);
//    TDK_Database::mv_PointCloudsVector.push_back(cloud->makeShared());
////    viewer->addPointCloud( cloud, "cloud2" );
////    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");

//    pcl::io::loadPLYFile ("D:\\MAIA\\SEM 1\\Software Engineering\\Project 3D-KORN\\eze_thresholded\\eze_2.ply", *cloud);
//    TDK_Database::mv_PointCloudsVector.push_back(cloud->makeShared());
////    viewer->addPointCloud( cloud, "cloud3" );
////    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud3");


//    pcl::io::loadPLYFile ("D:\\MAIA\\SEM 1\\Software Engineering\\Project 3D-KORN\\eze_thresholded\\eze_3.ply", *cloud);
//    TDK_Database::mv_PointCloudsVector.push_back(cloud->makeShared());
////    viewer->addPointCloud( cloud, "cloud4" );
////    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud4");


//    pcl::IterativeClosestPoint<pcl::PointXYZRGB,pcl::PointXYZRGB>::Ptr icp (new pcl::IterativeClosestPoint<pcl::PointXYZRGB,pcl::PointXYZRGB>);
//    //icp->setMaxCorrespondenceDistance (0.05);
//    icp->setMaximumIterations (50);
//    pcl::registration::IncrementalRegistration<pcl::PointXYZRGB> iicp;
//    iicp.setRegistration (icp);
//    int i = 0;
////    while (i < 4){

////      iicp.registerCloud (TDK_Database::mv_PointCloudsVector[i]);
////      transformPointCloud (*(TDK_Database::mv_PointCloudsVector[i]), *cloud, iicp.getAbsoluteTransform ());
////      viewer->addPointCloud( cloud, "cloud"+i );
////      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud"+i);


////      i++;
////    }

//    //viewer->addPointCloud( (TDK_Database::mv_PointCloudsVector)[1], "cloud" );
////    viewer->addPointCloud( cloud, "cloud" );
////    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
//    qvtkWidget->update();

//    //pcl::io::loadPLYFile ("D:\\MAIA\\SEM 1\\Software Engineering\\Project 3D-KORN\\testdata\\chair_alb0.ply", *cloud);
//    //viewer->addCoordinateSystem();
//    //if( !viewer->updatePointCloud( TDK_Database::mv_PointCloudsVector[2], "cloud" ) ){
//         // viewer->addPointCloud( (TDK_Database::mv_PointCloudsVector)[1], "cloud" );
////          viewer->addPointCloud( cloud, "cloud" );
////          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
////      // }
//    qvtkWidget->update();
//}

TDK_CentralWidget::~TDK_CentralWidget()
{

}
