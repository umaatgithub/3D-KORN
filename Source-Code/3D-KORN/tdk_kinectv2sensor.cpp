#include "tdk_kinectv2sensor.h"

TDK_KinectV2Sensor::TDK_KinectV2Sensor() : TDK_Sensor()
{
    mf_SetMvId(QString("KINECTV2"));
    mf_SetMvName(QString("Kinect V2"));
}

TDK_KinectV2Sensor::~TDK_KinectV2Sensor()
{

}

bool TDK_KinectV2Sensor::mf_IsAvailable()
{
    return true;
}

bool TDK_KinectV2Sensor::mf_SetupSensor()
{
    return true;
}

bool TDK_KinectV2Sensor::mf_StartSensor()
{
    return true;
}

bool TDK_KinectV2Sensor::mf_StopSensor()
{
    return true;
}
