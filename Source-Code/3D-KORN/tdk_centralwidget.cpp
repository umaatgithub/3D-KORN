#include "tdk_centralwidget.h"
#include <QDebug>
#include <pcl/registration/incremental_registration.h>

TDK_CentralWidget::TDK_CentralWidget(QWidget *parent) : QWidget(parent),
    mv_CentralGridLayout(new QGridLayout),
    mv_PointCloudQVTKWidget(new QVTKWidget)
{
    mf_setupUI();

}

TDK_CentralWidget::~TDK_CentralWidget()
{

}


void TDK_CentralWidget::mf_setupUI()
{
    vtkObject::GlobalWarningDisplayOff();

    mf_SetupPointCloudDisplayWidget();
    mf_SetupCropWidget();
    mf_SetupInformationWidget();
    mf_SetupPointCloudListWidget();
    mf_SetupPointCloudOperationsWidget();

    mv_CentralGridLayout->setColumnMinimumWidth(0, 300);
    mv_CentralGridLayout->setColumnMinimumWidth(1, 300);
    mv_CentralGridLayout->setColumnMinimumWidth(2, 300);

    mv_CentralGridLayout->setRowMinimumHeight(0, 300);
    mv_CentralGridLayout->setRowMinimumHeight(1, 300);

    mv_CentralGridLayout->setColumnStretch(1, 1);

    this->setLayout(mv_CentralGridLayout);
}

void TDK_CentralWidget::mf_SetupPointCloudDisplayWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Point Cloud Visualizer"));
    dockWidget->setWidget(mv_PointCloudQVTKWidget);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    mv_CentralGridLayout->addWidget(dockWidget, 0, 1, 2, 1);

    mv_PointCloudVisualizer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    mv_PointCloudVisualizer->setBackgroundColor (0.1, 0.1, 0.1);
    //mv_PointCloudStreamVisualizer->setCameraPosition( 0.0, 0.0, 2.5, 0.0, 0.0, 0.0 );

    mv_PointCloudVisualizer->addCoordinateSystem(1.0);
//    mv_PointCloudVisualizer->addCube(0, 1, 0, 1, 0, 1, 0, 0, 0,"cube");
//    mv_PointCloudVisualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");
//    mv_PointCloudVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, "cube");
//    mv_PointCloudVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.0, 0.0, "cube");
//    mv_PointCloudVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "cube");

    mv_PointCloudQVTKWidget->SetRenderWindow ( mv_PointCloudVisualizer->getRenderWindow () );
    mv_PointCloudVisualizer->setupInteractor ( mv_PointCloudQVTKWidget->GetInteractor (), mv_PointCloudQVTKWidget->GetRenderWindow ());

}

void TDK_CentralWidget::mf_SetupCropWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Crop Widget"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;



    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    mv_CentralGridLayout->addWidget(dockWidget, 0, 0);

}

void TDK_CentralWidget::mf_SetupInformationWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Information"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;

    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    mv_CentralGridLayout->addWidget(dockWidget, 1, 2);
}

void TDK_CentralWidget::mf_SetupPointCloudListWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Point Cloud List"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;



    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    mv_CentralGridLayout->addWidget(dockWidget, 0, 2);
}

void TDK_CentralWidget::mf_SetupPointCloudOperationsWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Point Cloud Operations"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;


    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    mv_CentralGridLayout->addWidget(dockWidget, 1, 0);
}
