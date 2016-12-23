#ifndef TDK_SENSOR_H
#define TDK_SENSOR_H

#include <QObject>
#include <QString>
#include <QDebug>
#include <map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class TDK_Sensor : public QObject
{
    Q_OBJECT
public:
    explicit TDK_Sensor(QObject *parent = 0);
    ~TDK_Sensor();

    virtual bool mf_IsAvailable() = 0;
    virtual bool mf_SetupSensor() = 0;
    virtual bool mf_StartSensor() = 0;
    virtual bool mf_StopSensor() = 0;

    std::map<QString, QString> mf_GetMvSensorDetails() const;
    void mf_SetMvSensorDetails(const std::map<QString, QString> &sensorDetails);

    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr mf_GetMvPointCloud() const;
    void mf_SetMvPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pointCloudPtr);

    QString mf_GetMvName() const;
    void mf_SetMvName(const QString &value);

    QString mf_GetMvId() const;
    void mf_SetMvId(const QString &value);

protected:
    QString mv_Id;
    QString mv_Name;
    std::map<QString, QString> mv_SensorDetails;
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr mv_PointCloud;

signals:
    void mf_SignalPointCloudUpdated();

public slots:
};

#endif // TDK_SENSOR_H
