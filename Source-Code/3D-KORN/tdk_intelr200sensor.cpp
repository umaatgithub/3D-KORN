#include "tdk_intelr200sensor.h"

TDK_IntelR200Sensor::TDK_IntelR200Sensor() : TDK_Sensor()
{
    mf_SetMvId(QString("INTELR200"));
    mf_SetMvName(QString("Intel R200"));

    mf_SetupSensor();
}

TDK_IntelR200Sensor::~TDK_IntelR200Sensor()
{

}

bool TDK_IntelR200Sensor::mf_IsAvailable()
{
    return true;
}

bool TDK_IntelR200Sensor::mf_SetupSensor()
{
    return true;
}

bool TDK_IntelR200Sensor::mf_StartSensor()
{
    return true;
}

bool TDK_IntelR200Sensor::mf_StopSensor()
{
    return true;
}
