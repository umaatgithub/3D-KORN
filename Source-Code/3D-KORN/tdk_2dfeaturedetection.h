#ifndef TDK_2DFEATUREDETECTION_H
#define TDK_2DFEATUREDETECTION_H

#include "kinect2_grabber.h"

#include <algorithm>
#include <math.h>

#include <opencv/highgui.h>
#include <opencv2/contrib/contrib.hpp>
// Feature detection function headers
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <QDebug>
#include <string>

/*!
 * \brief The TDK_2DFeatureDetection class
 *
 * The TDK_2DFeatureDetection class implements functions required for detecting features
 * and matching of point clouds using OpenCV library
 *
 * Use example
 * pcl::PointCloud<pcl::PointXYZRGB>::Ptr trainPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
 * pcl::PointCloud<pcl::PointXYZRGB>::Ptr queryPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
 *
 * pcl::PointCloud<pcl::PointXYZ>::Ptr trainKeyPoints(new pcl::PointCloud<pcl::PointXYZ>());
 * pcl::PointCloud<pcl::PointXYZ>::Ptr queryKeyPoints(new pcl::PointCloud<pcl::PointXYZ>());
 *
 * mv_2DFeatureDetectionPtr.setInputPointCloud(trainPointCloud);
 * mv_2DFeatureDetectionPtr.getMatchedFeatures(queryPointCloud, trainKeyPoints, queryKeyPoints);
 * mv_2DFeatureDetectionPtr.showMatchedFeatures3D(queryPointCloud, trainKeyPoints, queryKeyPoints)
 */

class TDK_2DFeatureDetection
{
public:

    TDK_2DFeatureDetection();
    ~TDK_2DFeatureDetection();

    void getMatchedFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &queryPc,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr &outKeyPointsTrain,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr &outKeyPointsQuery);
    void setInputPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inPointCloudPtr);
    void setInputPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &inPointCloud);

    void showKeyPoints(const cv::Mat &rgbImg, const std::vector<cv::KeyPoint> &keyPts);
    void showMatchedFeatures2D(const cv::Mat &rgb_1,
                               const std::vector<cv::KeyPoint> &keyPts_1,
                               const cv::Mat &rgb_2,
                               const std::vector<cv::KeyPoint> &keyPts_2,
                               const std::vector<cv::DMatch> &matches);
    void showMatchedFeatures3D(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &queryPC,
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr &trainKeyPoints,
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr &queryKeyPoints);

private:
    float maxVerticalShift = 0.3;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mv_TrainPointCloudPtr;
    pcl::Kinect2Grabber mv_Kinect2Grabber;

    void detectFeatures(const cv::Mat &rgbImg, std::vector<cv::KeyPoint> &keyPts);
    void getIntensityImage(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inPointCloud,
                           cv::Mat &outIntensityImage,
                           cv::Mat_<CameraSpacePoint> &outCameraSpaceMap);
    void matchFeatures(const cv::Mat &rgb_1,
                       std::vector<cv::KeyPoint> &keyPts_1,
                       const cv::Mat &rgb_2,
                       std::vector<cv::KeyPoint> &keyPts_2,
                       std::vector<cv::DMatch> &matches,
                       const int maxVertShiftPxls = INT_MAX);
    void getImageBoundaries(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inPointCloud,
                            ColorSpacePoint &maxColorCoords,
                            ColorSpacePoint &minColorCoords);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
            const std::string pointCloudId = "Reference point cloud");
};

#endif // TDK_2DFEATUREDETECTION_H
