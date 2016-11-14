#ifndef TDK_KINECT2GRABBER_H
#define TDK_KINECT2GRABBER_H
#include "kinect2_grabber.h"

typedef pcl::PointXYZRGB PointType;

class TDK_Kinect2Grabber
{
public:


    //default constructor for the TDK_Kinect2Grabber class
    TDK_Kinect2Grabber();

    //returns true if a new frame is available
    bool hasFrame();

    //You will change the address pointed by cloud, but if you dont pass by reference,
    //the pointer will not be updated in the calling code, it will point to the same dir.
    //fills the address of assigned cloud-pointer with a XYZRGB point cloud
    void getPointCloudFrame(pcl::PointCloud<PointType>::Ptr &cloud);


    // Gives you frame in im
    // void getRGBFrame(ImageType &im)

    // stores the reference to the qt widget that will be updated from the callback function for
    // RGBframe from grabber
    // setRGBWidget(...);

    // Same for depth image
    // setDepthDisplay(...);


private:
    //mutex
    boost::mutex mv_mutex;

    //point cloud
    pcl::PointCloud<PointType>::Ptr mv_cloud;



    boost::shared_ptr<pcl::Grabber> grabber;
    bool displayRGBSet;
    //Widget wRGB;
    bool displayDepthSet;


};

#endif // TDK_KINECT2GRABBER_H
