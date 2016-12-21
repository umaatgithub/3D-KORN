#include "tdk_scanwindow.h"

TDK_ScanWindow::TDK_ScanWindow(QWidget *parent) : QMainWindow(parent),
    mv_CentralWidget(new QWidget(this)),
    mv_CentralGridLayout(new QGridLayout),
    mv_SensorController(new TDK_SensorController),
    mv_SensorComboBox(new QComboBox),
    mv_PointCloudStreamQVTKWidget(new QVTKWidget),
    mv_XMinimumSpinBox(new QSpinBox),
    mv_XMaximumSpinBox(new QSpinBox),
    mv_YMinimumSpinBox(new QSpinBox),
    mv_YMaximumSpinBox(new QSpinBox),
    mv_ZMinimumSpinBox(new QSpinBox),
    mv_ZMaximumSpinBox(new QSpinBox)
{
    connect(mv_SensorComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(mf_SlotUpdateWindow(int)));
    connect(mv_XMinimumSpinBox, SIGNAL(valueChanged(int)), this, SLOT(mf_SlotUpdateBoundingBox()));
    connect(mv_XMaximumSpinBox, SIGNAL(valueChanged(int)), this, SLOT(mf_SlotUpdateBoundingBox()));
    connect(mv_YMinimumSpinBox, SIGNAL(valueChanged(int)), this, SLOT(mf_SlotUpdateBoundingBox()));
    connect(mv_YMaximumSpinBox, SIGNAL(valueChanged(int)), this, SLOT(mf_SlotUpdateBoundingBox()));
    connect(mv_ZMinimumSpinBox, SIGNAL(valueChanged(int)), this, SLOT(mf_SlotUpdateBoundingBox()));
    connect(mv_ZMaximumSpinBox, SIGNAL(valueChanged(int)), this, SLOT(mf_SlotUpdateBoundingBox()));
}

void TDK_ScanWindow::mf_setupUI()
{
    vtkObject::GlobalWarningDisplayOff();
    this->setWindowFlags(Qt::Dialog);
    resize(300, 400);
    setCentralWidget(mv_CentralWidget);

    mf_SetupPointCloudStreamWidget();
    mf_SetupSensorWidget();
    mf_SetupVideoStreamWidget();
    mf_SetupDepthMapWidget();
    mf_SetupPointcloudListWidget();

    mv_CentralGridLayout->setColumnMinimumWidth(0, 300);
    mv_CentralGridLayout->setColumnMinimumWidth(1, 300);
    mv_CentralGridLayout->setColumnMinimumWidth(2, 300);

    mv_CentralGridLayout->setRowMinimumHeight(0, 300);
    mv_CentralGridLayout->setRowMinimumHeight(1, 300);

    mv_CentralGridLayout->setColumnStretch(1, 1);

    mv_CentralWidget->setLayout(mv_CentralGridLayout);
}

void TDK_ScanWindow::mf_SetupPointCloudStreamWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Point Cloud Visualizer"));
    dockWidget->setWidget(mv_PointCloudStreamQVTKWidget);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    mv_CentralGridLayout->addWidget(dockWidget, 0, 1, 2, 1);

    mv_PointCloudStreamVisualizer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    mv_PointCloudStreamVisualizer->setBackgroundColor (0.1, 0.1, 0.1);


    mv_PointCloudStreamVisualizer->addCube(0, 1, 0, 1, 0, 1);
    mv_PointCloudStreamVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "cube");
//    mv_PointCloudStreamVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "cube");
    mv_PointCloudStreamVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "cube");

    mv_PointCloudStreamQVTKWidget->SetRenderWindow ( mv_PointCloudStreamVisualizer->getRenderWindow () );
    mv_PointCloudStreamVisualizer->setupInteractor ( mv_PointCloudStreamQVTKWidget->GetInteractor (), mv_PointCloudStreamQVTKWidget->GetRenderWindow ());
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

    mv_SensorComboBox->setFixedHeight(22);
    while(it != sensorNames.end()){
        mv_SensorComboBox->addItem(it->second, it->first);
        it++;
    }


    mv_XMinimumSpinBox->setRange(-10, 10);
    mv_XMinimumSpinBox->setSingleStep(1);
    mv_XMinimumSpinBox->setValue(0);
    mv_XMinimumSpinBox->setFixedWidth(60);
    mv_XMinimumSpinBox->setFixedHeight(22);
    mv_XMinimumSpinBox->setSuffix(QString("m"));


    mv_XMaximumSpinBox->setRange(-10, 10);
    mv_XMaximumSpinBox->setSingleStep(1);
    mv_XMaximumSpinBox->setValue(1);
    mv_XMaximumSpinBox->setFixedWidth(60);
    mv_XMaximumSpinBox->setFixedHeight(22);
    mv_XMaximumSpinBox->setSuffix(QString("m"));

    mv_YMinimumSpinBox->setRange(-10, 10);
    mv_YMinimumSpinBox->setSingleStep(1);
    mv_YMinimumSpinBox->setValue(0);
    mv_YMinimumSpinBox->setFixedHeight(22);
    mv_YMinimumSpinBox->setSuffix(QString("m"));

    mv_YMaximumSpinBox->setRange(-10, 10);
    mv_YMaximumSpinBox->setSingleStep(1);
    mv_YMaximumSpinBox->setValue(1);
    mv_YMaximumSpinBox->setFixedHeight(22);
    mv_YMaximumSpinBox->setSuffix(QString("m"));

    mv_ZMinimumSpinBox->setRange(-10, 10);
    mv_ZMinimumSpinBox->setSingleStep(1);
    mv_ZMinimumSpinBox->setValue(0);
    mv_ZMinimumSpinBox->setFixedHeight(22);
    mv_ZMinimumSpinBox->setSuffix(QString("m"));

    mv_ZMaximumSpinBox->setRange(-10, 10);
    mv_ZMaximumSpinBox->setSingleStep(1);
    mv_ZMaximumSpinBox->setValue(1);
    mv_ZMaximumSpinBox->setFixedHeight(22);
    mv_ZMaximumSpinBox->setSuffix(QString("m"));

    QPushButton *startScanPushButton = new QPushButton(QString("START SCAN"));
    startScanPushButton->setFixedHeight(22);
    QPushButton *stopScanPushButton = new QPushButton(QString("STOP SCAN"));
    stopScanPushButton->setFixedHeight(22);

    QCheckBox *registerationCheckBox = new QCheckBox;
    registerationCheckBox->setText(QString("Register points clouds during scan"));

    gridLayout->addWidget(sensorLabel, 0, 0, 1, 2);
    gridLayout->addWidget(mv_SensorComboBox, 0, 2, 1, 2);
    gridLayout->addWidget(new QLabel(QString("x-minimum : ")), 1, 0);
    gridLayout->addWidget(mv_XMinimumSpinBox, 1, 1);
    gridLayout->addWidget(new QLabel(QString("x-maximum : ")), 1, 2);
    gridLayout->addWidget(mv_XMaximumSpinBox, 1, 3);
    gridLayout->addWidget(new QLabel(QString("y-minimum : ")), 2, 0);
    gridLayout->addWidget(mv_YMinimumSpinBox, 2, 1);
    gridLayout->addWidget(new QLabel(QString("y-maximum : ")), 2, 2);
    gridLayout->addWidget(mv_YMaximumSpinBox, 2, 3);
    gridLayout->addWidget(new QLabel(QString("z-minimum : ")), 3, 0);
    gridLayout->addWidget(mv_ZMinimumSpinBox, 3, 1);
    gridLayout->addWidget(new QLabel(QString("z-maximum : ")), 3, 2);
    gridLayout->addWidget(mv_ZMaximumSpinBox, 3, 3);
    gridLayout->addWidget(registerationCheckBox, 4, 0, 1, 4);
    gridLayout->addWidget(startScanPushButton, 5, 0, 1, 2);
    gridLayout->addWidget(stopScanPushButton, 5, 2, 1, 2);

    gridLayout->setRowMinimumHeight(0, 30);
    gridLayout->setHorizontalSpacing(10);
    gridLayout->setVerticalSpacing(15);
    gridLayout->setMargin(12);

    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);

    mv_CentralGridLayout->addWidget(dockWidget, 0, 0);

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
    mv_CentralGridLayout->addWidget(dockWidget, 0, 2);
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
    mv_CentralGridLayout->addWidget(dockWidget, 1, 2);
}

void TDK_ScanWindow::mf_SetupPointcloudListWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Platform parameters"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;

    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    mv_CentralGridLayout->addWidget(dockWidget, 1, 0);
}

void TDK_ScanWindow::mf_SlotUpdatePointCloudStreamWidget()
{
    if( !mv_PointCloudStreamVisualizer->updatePointCloud( mv_Sensor->mf_GetMvPointCloud(), "cloud" ) ){
        mv_PointCloudStreamVisualizer->addPointCloud( mv_Sensor->mf_GetMvPointCloud(), "cloud" );
        mv_PointCloudStreamVisualizer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    }
    mv_PointCloudStreamQVTKWidget->update();
}

void TDK_ScanWindow::mf_SlotUpdateBoundingBox()
{
    mv_PointCloudStreamVisualizer->removeAllShapes();
    mv_PointCloudStreamVisualizer->addCube(mv_XMinimumSpinBox->value(), mv_XMaximumSpinBox->value(), mv_YMinimumSpinBox->value(), mv_YMaximumSpinBox->value(), mv_ZMinimumSpinBox->value(), mv_ZMaximumSpinBox->value());
    mv_PointCloudStreamVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "cube");
//    mv_PointCloudStreamVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "cube");
    mv_PointCloudStreamVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "cube");
    mv_PointCloudStreamQVTKWidget->update();

}

void TDK_ScanWindow::mf_SlotUpdateWindow(int sensorIndex)
{
    QString sensorName = mv_SensorComboBox->itemText(sensorIndex);
    mv_Sensor = mv_SensorController->mf_GetSensor(sensorName);
    qDebug() << mv_Sensor->mf_GetMvName();
    connect(mv_Sensor, SIGNAL(mf_SignalPointCloudUpdated()), this, SLOT(mf_SlotUpdatePointCloudStreamWidget()));
    mv_Sensor->mf_StartSensor();
}
