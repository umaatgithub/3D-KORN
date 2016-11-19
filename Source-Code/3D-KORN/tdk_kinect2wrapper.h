#ifndef TDK_KINECT2WRAPPER_H
#define TDK_KINECT2WRAPPER_H

#include <QObject>
#include <QDebug>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "kinect2_grabber.h"

class TDK_Kinect2Wrapper : public QObject
{
    Q_OBJECT
public:
    explicit TDK_Kinect2Wrapper(QObject *parent = 0);
    void startKinect();
    boost::mutex mutex;

    boost::shared_ptr<pcl::Grabber> mv_grabber;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mv_cloud;
    boost::function<void( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& )> mv_pointCloudCallback =
            [this]( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& ptr ){
        qDebug("mv_pointCloudCallback --------------------------");
        boost::mutex::scoped_lock lock(mutex);
        setMv_cloud(ptr);

    };

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getMv_cloud() const;
    void setMv_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &value);

signals:
    void signalCloudUpdated();

public slots:

};

#endif // TDK_KINECT2WRAPPER_H
