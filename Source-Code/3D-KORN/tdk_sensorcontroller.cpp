#include "tdk_sensorcontroller.h"

TDK_SensorController::TDK_SensorController(QObject *parent) : QObject(parent)
{
    TDK_Sensor *sensor = new TDK_KinectV2Sensor();
    mv_Sensors[sensor->mf_GetMvName()] = sensor;
//    sensor = new TDK_IntelR200Sensor();
//    mv_Sensors[sensor->mf_GetMvName()] = sensor;
}

TDK_SensorController::~TDK_SensorController()
{

}

bool TDK_SensorController::mf_IsSensorAvailable()
{
    std::map<QString, TDK_Sensor*>::iterator it = mv_Sensors.begin();
    while(it != mv_Sensors.end()){
        if(((TDK_Sensor *)it->second)->mf_IsAvailable()){
            return true;
        }
        it++;
    }
    return false;
}

std::map<QString, QString> TDK_SensorController::mf_GetAvailableSensorNames()
{
    std::map<QString, TDK_Sensor*>::iterator it = mv_Sensors.begin();
    std::map<QString, QString> sensorNames;
    while(it != mv_Sensors.end()){
        if( ((TDK_Sensor *)it->second)->mf_IsAvailable() ){
            sensorNames[it->first] = ((TDK_Sensor *)it->second)->mf_GetMvName();
        }
        it++;    
    }
    return sensorNames;
}

TDK_Sensor *TDK_SensorController::mf_GetSensor(QString sensorId)
{
    return mv_Sensors[sensorId];
}
