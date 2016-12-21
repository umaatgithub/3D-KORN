#ifndef TDK_CENTRALWIDGET_H
#define TDK_CENTRALWIDGET_H

#include <QWidget>
#include <QGridLayout>
#include <QDockWidget>
#include <QScrollArea>
#include <QLabel>
#include <QPushButton>
#include <QVTKWidget.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/registration/icp.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>

#include "tdk_database.h"

//typedef pcl::PointXYZRGBA PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;

class TDK_CentralWidget : public QWidget
{
    Q_OBJECT
public:
    explicit TDK_CentralWidget(QWidget *parent = 0);
    ~TDK_CentralWidget();

private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
//    PointCloudT::Ptr cloudSource2;
//    PointCloudT::Ptr cloudRegistered;
    /** @brief 0 = x | 1 = y | 2 = z */
    int filtering_axis;

    /** @brief Holds the color mode for @ref colorCloudDistances */
    int color_mode;
    QGridLayout *layout;
    QVTKWidget *qvtkWidget;


signals:

public slots:
};

#endif // TDK_CENTRALWIDGET_H
