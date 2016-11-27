#include "tdk_scanwindow.h"

TDK_ScanWindow::TDK_ScanWindow(QWidget *parent) : QMainWindow(parent),
    centralWidget(new QWidget(this)),
    gridLayoutCentralWidget(new QGridLayout),
    mv_SensorController(new TDK_SensorController),
    qvtkWidget(new QVTKWidget)
{
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
    dockWidget->setWidget(qvtkWidget);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    gridLayoutCentralWidget->addWidget(dockWidget, 0, 1, 2, 1);

    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    viewer->setBackgroundColor (0.1, 0.1, 0.1);
    qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor ( qvtkWidget->GetInteractor (), qvtkWidget->GetRenderWindow ());
}

void TDK_ScanWindow::mf_SetupSensorWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Sensor"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;

    QLabel *sensorLabel = new QLabel("Select sensor : ");
    sensorLabel->setFixedHeight(22);

    std::map<QString, QString> sensorNames = mv_SensorController->mf_GetAvailableSensorNames();
    std::map<QString, QString>::iterator it = sensorNames.begin();
    QComboBox *sensorComboBox = new QComboBox;
    sensorComboBox->setFixedHeight(22);
    sensorComboBox->setFixedWidth(170);
    while(it != sensorNames.end()){
        sensorComboBox->addItem(it->second, it->first);
        it++;
    }

    gridLayout->addWidget(sensorLabel, 0, 0);
    gridLayout->addWidget(sensorComboBox, 0, 1);

    gridLayout->setRowMinimumHeight(0, 30);
    gridLayout->setColumnMinimumWidth(0, 100);
    gridLayout->setColumnStretch(1, 1);

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

void TDK_ScanWindow::slotUpdateSensorOutputWidget()
{
//    if( !viewer->updatePointCloud( tdk_Kinect2Wrapper->getMv_cloud(), "cloud" ) ){
//        viewer->addPointCloud( tdk_Kinect2Wrapper->getMv_cloud(), "cloud" );
//        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
//    }
//    qvtkWidget->update();
}
