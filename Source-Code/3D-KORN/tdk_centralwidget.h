#ifndef TDK_CENTRALWIDGET_H
#define TDK_CENTRALWIDGET_H

#include <QWidget>
#include <QGridLayout>
#include <QDockWidget>
#include <QScrollArea>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QVTKWidget.h>
#include <QListWidget>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>

#include "tdk_database.h"

class TDK_CentralWidget : public QWidget
{
    Q_OBJECT
public:
    explicit TDK_CentralWidget(QWidget *parent = 0);
    ~TDK_CentralWidget();

private:

    QGridLayout *mv_CentralGridLayout;
    QVTKWidget *mv_PointCloudQVTKWidget;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> mv_PointCloudVisualizer;

    //Point cloud operations
    QComboBox *mv_MeshAlgorithmComboBox;
    QPushButton *mv_GenerateMeshPushButton;
    QComboBox *mv_RegistrationComboBox;
    QPushButton *mv_RegistrationPushButton;

    //Selection widget
    QTabWidget *mv_SelectionTabWidget;
    QListWidget *mv_PointCloudListTab;
    QListWidget *mv_RegisteredPointCloudListTab;
    QListWidget *mv_MeshListTab;


    void mf_setupUI();
    void mf_SetupPointCloudDisplayWidget();
    void mf_SetupCropWidget();
    void mf_SetupInformationWidget();
    void mf_SetupSelectionWidget();
    void mf_SetupPointCloudOperationsWidget();

signals:
    void mf_SignalPointCloudListUpdated();
    void mf_SignalRegisteredPointCloudListUpdated();
    void mf_SignalMeshListUpdated();

public slots:
    void mf_SlotRegisterPointCloud();
    void mf_SlotGenerateMesh();

    void mf_SlotUpdatePointCloudListTab();
    void mf_SlotUpdateRegisteredPointCloudListTab();
    void mf_SlotUpdateMeshListTab();
};

#endif // TDK_CENTRALWIDGET_H
