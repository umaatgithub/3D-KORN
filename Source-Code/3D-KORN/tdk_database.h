#ifndef TDK_DATABASE_H
#define TDK_DATABASE_H

#include <QObject>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "tdk_edit.h"

class TDK_Database : public QObject
{
    Q_OBJECT
public:
    explicit TDK_Database(QObject *parent = 0);
    ~TDK_Database();

    static std::vector<TDK_Edit*> mv_EditHistoryVector;
    static std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> mv_PointCloudsVector;

signals:

public slots:

};

#endif // TDK_DATABASE_H
