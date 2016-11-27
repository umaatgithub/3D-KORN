#ifndef TDK_INTELR200SENSOR_H
#define TDK_INTELR200SENSOR_H

#include "tdk_sensor.h"

class TDK_IntelR200Sensor : public TDK_Sensor
{
public:
    TDK_IntelR200Sensor();
    ~TDK_IntelR200Sensor();

    bool mf_IsAvailable();
    bool mf_SetupSensor();
    bool mf_StartSensor();
    bool mf_StopSensor();

};

#endif // TDK_INTELR200SENSOR_H
