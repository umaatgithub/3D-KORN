#ifndef TDK_SCANWINDOW_H
#define TDK_SCANWINDOW_H

// Include Qt headers
#include <QMainWindow>
#include <QGridLayout>
#include <QDockWidget>
#include <QScrollArea>
#include <QLabel>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QCheckBox>
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
#include "tdk_database.h"

class TDK_ScanWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit TDK_ScanWindow(QWidget *parent = 0);
    QWidget *mv_CentralWidget;
    QGridLayout *mv_CentralGridLayout;
    TDK_SensorController *mv_SensorController;
    TDK_Sensor *mv_Sensor;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> mv_PointCloudStreamVisualizer;
    QVTKWidget *mv_PointCloudStreamQVTKWidget;


    //Flag variables
    bool mv_FlagRegisterDuringScan;
    bool mv_FlagScanning;

    //Sensor widgets
    QComboBox *mv_SensorComboBox;
    QDoubleSpinBox *mv_XMinimumSpinBox;
    QDoubleSpinBox *mv_XMaximumSpinBox;
    QDoubleSpinBox *mv_YMinimumSpinBox;
    QDoubleSpinBox *mv_YMaximumSpinBox;
    QDoubleSpinBox *mv_ZMinimumSpinBox;
    QDoubleSpinBox *mv_ZMaximumSpinBox;
    QPushButton *mv_StartScanPushButton;
    QPushButton *mv_StopScanPushButton;
    QCheckBox *mv_RegisterationCheckBox;


    void mf_setupUI();
    void mf_SetupPointCloudStreamWidget();
    void mf_SetupSensorWidget();
    void mf_SetupVideoStreamWidget();
    void mf_SetupDepthMapWidget();
    void mf_SetupPointcloudListWidget();

    //enum SensorType{ KINECTV1=0, KINECTV2, INTELR200};

signals:

public slots:
    void mf_SlotUpdateWindow(int sensorIndex);
    void mf_SlotUpdateBoundingBox();
    void mf_SlotPointCloudRegistration(bool flagRegisterDuringScan);
    void mf_SlotStartScan();
    void mf_SlotStopScan();

    void mf_SlotUpdatePointCloudStream();

};

#endif // TDK_SCANWINDOW_H
