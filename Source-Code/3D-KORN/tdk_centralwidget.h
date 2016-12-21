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

#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class TDK_CentralWidget : public QWidget
{
    Q_OBJECT
public:
    explicit TDK_CentralWidget(QWidget *parent = 0);

private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    PointCloudT::Ptr cloud;
    /** @brief 0 = x | 1 = y | 2 = z */
    int filtering_axis;

    /** @brief Holds the color mode for @ref colorCloudDistances */
    int color_mode;
    QGridLayout *layout;


signals:

public slots:
};

#endif // TDK_CENTRALWIDGET_H
