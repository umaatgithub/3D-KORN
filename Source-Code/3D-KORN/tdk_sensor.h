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

    virtual bool    mf_IsAvailable      () = 0;
    virtual bool    mf_SetupSensor      () = 0;
    virtual bool    mf_StartSensor      () = 0;
    virtual bool    mf_StopSensor       () = 0;


    void    mf_SetMvSensorDetails       (const std::map<QString, QString> &sensorDetails);
    void    mf_SetMvPointCloud          (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pointCloudPtr);
    void    mf_SetMvName                (const QString &value);
    void    mf_SetMvId                  (const QString &value);
    void    mf_SetMvFlagFilterPoints    (bool value);

    void    mf_SetFilterBox             (float xmin, float xmax, float ymin, float ymax, float zmin, float zmax);

    std::map<QString, QString>                  mf_GetMvSensorDetails       () const;
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr mf_GetMvPointCloud          () const;
    QString                                     mf_GetMvName                () const;
    QString                                     mf_GetMvId                  () const;
    bool                                        mf_GetMvFlagFilterPoints    () const;

protected:
    QString mv_Id;
    QString mv_Name;

    std::map<QString, QString>                      mv_SensorDetails;
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr     mv_PointCloud;

    bool    mv_FlagFilterPoints;
    float   mv_XMin, mv_XMax;
    float   mv_YMin, mv_YMax;
    float   mv_ZMin, mv_ZMax;

signals:
    void    mf_SignalPointCloudUpdated  ();
    void    mf_SignalFlagFilterUpdated  ();
    void    mf_SignalFilterBoxUpdated   ();

public slots:
};

#endif // TDK_SENSOR_H
