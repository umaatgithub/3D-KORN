#include "tdk_database.h"

std::vector<TDK_Edit*> TDK_Database::mv_EditHistoryVector ;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> TDK_Database::mv_PointCloudsVector ;
std::vector<float> TDK_Database::mv_PointCloudsRotation;

TDK_Database::TDK_Database(QObject *parent) : QObject(parent)
{

}

TDK_Database::~TDK_Database()
{

}
