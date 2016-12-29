#ifndef TDK_KINECT2GRABBER_H
#define TDK_KINECT2GRABBER_H
#include "kinect2_grabber.h"
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB PointType;

class TDK_Kinect2Grabber
{
public:

    //default constructor for the TDK_Kinect2Grabber class
    TDK_Kinect2Grabber();
    ~TDK_Kinect2Grabber() {}

    void mf_closeKinect();

    //You will change the address pointed by cloud, but if you dont pass by reference,
    //the pointer will not be updated in the calling code, it will point to the same dir.
    //fills the address of assigned cloud-pointer with a XYZRGB point cloud
    void mf_getPointCloudFrame(pcl::PointCloud<PointType>::Ptr &cloud);

    void mf_SetScanBoxLimits(int& x_min,int& x_max,int& y_min,int& y_max,int& z_min,int& z_max);

    //TDK image viewer//
    pcl::visualization::ImageViewer::Ptr mv_imageViewer;

    //void mf_getRGBImage(pcl::PointCloud<PointType>::Ptr &cloud);

private:
    //mutex
    boost::mutex mv_mutex;
    boost::shared_ptr<pcl::Kinect2Grabber> mv_grabber;

    //point cloud
    pcl::PointCloud<PointType>::Ptr mv_cloud;

    //TDK pcl cloud viewer
    pcl::visualization::PCLVisualizer::Ptr mv_cloudViewer;


};

#endif // TDK_KINECT2GRABBER_H
