#ifndef TDK_SENSORCONTROLLER_H
#define TDK_SENSORCONTROLLER_H

#include <QObject>
#include <map>

#include "tdk_kinectv2sensor.h"
#include "tdk_intelr200sensor.h"

class TDK_SensorController : public QObject
{
    Q_OBJECT
public:
    explicit TDK_SensorController(QObject *parent = 0);
    ~TDK_SensorController();

    bool mf_IsSensorAvailable() const;
    std::map<QString, QString> mf_GetAvailableSensorNames();
    TDK_Sensor *mf_GetSensor(QString sensorId);

protected:
    std::map<QString, TDK_Sensor*> mv_Sensors;

signals:

public slots:
};

#endif // TDK_SENSORCONTROLLER_H
