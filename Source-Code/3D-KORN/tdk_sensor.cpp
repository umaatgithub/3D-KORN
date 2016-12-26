#include "tdk_sensor.h"

TDK_Sensor::TDK_Sensor(QObject *parent) : QObject(parent)
{

}

TDK_Sensor::~TDK_Sensor()
{

}

std::map<QString, QString> TDK_Sensor::mf_GetMvSensorDetails() const
{
    return mv_SensorDetails;
}

void TDK_Sensor::mf_SetMvSensorDetails(const std::map<QString, QString> &sensorDetails)
{
    mv_SensorDetails = sensorDetails;
}

pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr TDK_Sensor::mf_GetMvPointCloud() const
{
    return mv_PointCloud;
}

void TDK_Sensor::mf_SetMvPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pointCloudPtr)
{
    //qDebug() << "Setting point cloud";
    mv_PointCloud = pointCloudPtr->makeShared();
    emit mf_SignalPointCloudUpdated();
}

QString TDK_Sensor::mf_GetMvName() const
{
    return mv_Name;
}

void TDK_Sensor::mf_SetMvName(const QString &value)
{
    mv_Name = value;
}

QString TDK_Sensor::mf_GetMvId() const
{
    return mv_Id;
}

void TDK_Sensor::mf_SetMvId(const QString &value)
{
    mv_Id = value;
}
