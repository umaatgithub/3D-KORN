#include "tdk_sensor.h"

TDK_Sensor::TDK_Sensor(QObject *parent) : QObject(parent),
    mv_FlagFilterPoints(false),
    mv_XMin ( -0.5 ),
    mv_XMax (  0.5 ),
    mv_YMin ( -1.5 ),
    mv_YMax (  1.0 ),
    mv_ZMin (  2.0 ),
    mv_ZMax (  3.0 )
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
    if(pointCloudPtr != nullptr && pointCloudPtr->points.size() > 0){
        mv_PointCloud = pointCloudPtr->makeShared();
        emit mf_SignalPointCloudUpdated();
    }
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

bool TDK_Sensor::mf_GetMvFlagFilterPoints() const
{
    return mv_FlagFilterPoints;
}

void TDK_Sensor::mf_SetMvFlagFilterPoints(bool value)
{
    mv_FlagFilterPoints = value;
    emit mf_SignalFlagFilterUpdated();
}

void TDK_Sensor::mf_SetFilterBox(float xmin, float xmax, float ymin, float ymax, float zmin, float zmax)
{
    mv_XMin = xmin;
    mv_XMax = xmax;
    mv_YMin = ymin;
    mv_YMax = ymax;
    mv_ZMin = zmin;
    mv_ZMax = zmax;
    emit mf_SignalFilterBoxUpdated();
}
