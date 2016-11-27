#ifndef TDK_SCANWINDOW_H
#define TDK_SCANWINDOW_H

// Include Qt headers
#include <QMainWindow>
#include <QGridLayout>
#include <QDockWidget>
#include <QScrollArea>
#include <QLabel>
#include <QComboBox>
#include <map>
#include <QDebug>

//Include PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>

#include "tdk_sensorcontroller.h"

class TDK_ScanWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit TDK_ScanWindow(QWidget *parent = 0);
    QWidget *centralWidget;
    QGridLayout *gridLayoutCentralWidget;
    TDK_SensorController *mv_SensorController;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    QVTKWidget *qvtkWidget;

    void mf_SetupSensorOutputWidget();
    void mf_SetupSensorWidget();
    void mf_SetupVideoStreamWidget();
    void mf_SetupDepthMapWidget();
    void mf_SetupPointcloudListWidget();

    //enum SensorType{ KINECTV1=0, KINECTV2, INTELR200};

signals:

public slots:
    void slotUpdateSensorOutputWidget();

};

#endif // TDK_SCANWINDOW_H
