#ifndef TDK_KINECTV2SENSOR_H
#define TDK_KINECTV2SENSOR_H

#include "tdk_sensor.h"

class TDK_KinectV2Sensor : public TDK_Sensor
{
public:
    TDK_KinectV2Sensor();
    ~TDK_KinectV2Sensor();

    bool mf_IsAvailable();
    bool mf_SetupSensor();
    bool mf_StartSensor();
    bool mf_StopSensor();

};

#endif // TDK_KINECTV2SENSOR_H
