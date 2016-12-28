#ifndef TDK_SCANWINDOW_H
#define TDK_SCANWINDOW_H

// Include Qt headers
#include <QMainWindow>
#include <QStatusBar>
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
#include <QRadioButton>
#include <QKeyEvent>

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
#include "tdk_scanregistration.h"

class TDK_ScanWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit TDK_ScanWindow(QWidget *parent = 0);
    ~TDK_ScanWindow();
    QWidget *mv_CentralWidget;
    QStatusBar *mv_StatusBar;
    QGridLayout *mv_CentralGridLayout;
    TDK_SensorController *mv_SensorController;
    TDK_Sensor *mv_Sensor;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> mv_PointCloudStreamVisualizer;
    QVTKWidget *mv_PointCloudStreamQVTKWidget;
    unsigned int mv_NumberOfPointCloudsCaptured;


    //Flag variables
    bool mv_FlagRegisterDuringScan;
    bool mv_FlagScanning;
    bool mv_FlagTurnTableParametersEnabled;
    bool mv_FlagPointCloudExists;

    //Sensor widgets
    QComboBox *mv_SensorComboBox;
    QDoubleSpinBox *mv_XMinimumSpinBox;
    QDoubleSpinBox *mv_XMaximumSpinBox;
    QDoubleSpinBox *mv_YMinimumSpinBox;
    QDoubleSpinBox *mv_YMaximumSpinBox;
    QDoubleSpinBox *mv_ZMinimumSpinBox;
    QDoubleSpinBox *mv_ZMaximumSpinBox;
    QCheckBox *mv_RegisterationCheckBox;
    QPushButton *mv_StartScanPushButton;
    QPushButton *mv_StopScanPushButton;
    QLabel *mv_NumberOfPointCloudsCapturedLabel;
    QPushButton *mv_CapturePointCloudPushButton;

    //Platform parameters widgets
    QRadioButton *mv_PlatformParametersYesRadioButton;
    QRadioButton *mv_PlatformParametersNoRadioButton;
    QDoubleSpinBox *mv_IncrementalRotationAngleSpinBox;
    QDoubleSpinBox *mv_NumberOfRotationsSpinBox;

    void mf_setupUI();
    void mf_SetupPointCloudStreamWidget();
    void mf_SetupSensorWidget();
    void mf_SetupVideoStreamWidget();
    void mf_SetupDepthMapWidget();
    void mf_SetupPlatformParametersWidget();

signals:
    void mf_SignalStatusChanged(QString, QColor);

public slots:
    void mf_SlotUpdateWindow(int sensorIndex);
    void mf_SlotUpdateBoundingBox();
    void mf_SlotPointCloudRegistration(bool flagRegisterDuringScan);
    void mf_SlotStartScan();
    void mf_SlotStopScan();

    void mf_SlotHandlePlatformParameters(bool flagEnablePlatformParameters);

    void mf_SlotUpdatePointCloudStream();
    void mf_SlotCapturePointCloud(float degreesRotated);
    void mf_SlotCapturePointCloudButtonClick();

    void mf_SlotUpdateStatusBar(QString status, QColor statusColor);

};

#endif // TDK_SCANWINDOW_H
