#include "tdk_kinectv2sensor.h"

TDK_KinectV2Sensor::TDK_KinectV2Sensor() : TDK_Sensor()
{
    mf_SetMvId(QString("KINECTV2"));
    mf_SetMvName(QString("Kinect V2"));

    mf_SetupSensor();
}

TDK_KinectV2Sensor::~TDK_KinectV2Sensor()
{

}

bool TDK_KinectV2Sensor::mf_IsAvailable()
{
    // TO BE MODIFIED
    qDebug() << mv_Grabber->isAvailable();
    if(mv_Grabber->isAvailable()){
        return true;
    }
    return false;
}

bool TDK_KinectV2Sensor::mf_SetupSensor()
{
    qDebug() << "Setup sensor";
    mv_Grabber = boost::make_shared<pcl::Kinect2Grabber>();
    mv_Connection = mv_Grabber->registerCallback( mv_PointCloudCallback );
    qDebug() << "Setup done";
    return true;
}

bool TDK_KinectV2Sensor::mf_StartSensor()
{
    if(mf_IsAvailable()){
        qDebug() << "Kinect started";
        mv_Grabber->start();
        return true;
    }
    return false;
}

bool TDK_KinectV2Sensor::mf_StopSensor()
{
    if(mf_IsAvailable()){
        mv_Grabber->stop();
        return true;
    }
    return false;
}
