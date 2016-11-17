#include "tdk_scanwindow.h"

TDK_ScanWindow::TDK_ScanWindow(QWidget *parent) : QMainWindow(parent),
    centralWidget(new QWidget(this)),
    gridLayoutCentralWidget(new QGridLayout)
{
    connect(tdk_Kinect2Wrapper, SIGNAL(signalCloudUpdated(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& ptr)),
            this, SLOT(slotUpdateSensorOutputWidget(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr)));
    vtkObject::GlobalWarningDisplayOff();
    this->setWindowFlags(Qt::Dialog);
    resize(300, 400);
    setCentralWidget(centralWidget);

    mf_SetupSensorOutputWidget();
    mf_SetupSensorWidget();
    mf_SetupVideoStreamWidget();
    mf_SetupDepthMapWidget();
    mf_SetupPointcloudListWidget();

    gridLayoutCentralWidget->setColumnMinimumWidth(0, 300);
    gridLayoutCentralWidget->setColumnMinimumWidth(1, 300);
    gridLayoutCentralWidget->setColumnMinimumWidth(2, 300);

    gridLayoutCentralWidget->setRowMinimumHeight(0, 300);
    gridLayoutCentralWidget->setRowMinimumHeight(1, 300);

    gridLayoutCentralWidget->setColumnStretch(1, 1);

    centralWidget->setLayout(gridLayoutCentralWidget);
}

void TDK_ScanWindow::mf_SetupSensorOutputWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Sensor Output"));
    QVTKWidget *qvtkWidget = new QVTKWidget;

    dockWidget->setWidget(qvtkWidget);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);

    gridLayoutCentralWidget->addWidget(dockWidget, 0, 1, 2, 1);

    //cloudSource1.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //pcl::io::loadPLYFile ("D:\\MAIA\\SEM 1\\Software Engineering\\Project 3D-KORN\\eze_thresholded\\eze_3.ply", *cloudSource1);

    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    viewer->setBackgroundColor (0.1, 0.1, 0.1);
    qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor ( qvtkWidget->GetInteractor (), qvtkWidget->GetRenderWindow ());
    //viewer->addPointCloud (cloudSource1, "cloud");
    //viewer->resetCamera ();
    //qvtkWidget->update ();
    //viewer->spinOnce();
    //pcl::io::loadPLYFile ("D:\\MAIA\\SEM 1\\Software Engineering\\Project 3D-KORN\\eze_thresholded\\eze_1.ply", *cloudSource1);
    //viewer->updatePointCloud(cloudSource1, "cloud");

}

void TDK_ScanWindow::mf_SetupSensorWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Sensor"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;

    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);

    gridLayoutCentralWidget->addWidget(dockWidget, 0, 0);

}

void TDK_ScanWindow::mf_SetupVideoStreamWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Video Stream"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;

    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);

    gridLayoutCentralWidget->addWidget(dockWidget, 0, 2);
}

void TDK_ScanWindow::mf_SetupDepthMapWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Depth Map"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;

    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);

    gridLayoutCentralWidget->addWidget(dockWidget, 1, 2);
}

void TDK_ScanWindow::mf_SetupPointcloudListWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Pointcloud Explorer"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;

    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);

    gridLayoutCentralWidget->addWidget(dockWidget, 1, 0);
}

void TDK_ScanWindow::slotUpdateSensorOutputWidget(const pcl::PointCloud::ConstPtr &ptr)
{
    if( !viewer->updatePointCloud( ptr, "cloud" ) ){
        viewer->addPointCloud( ptr, "cloud" );
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    }
}
