#include "tdk_2dfeaturedetection.h"
#include "tdk_scanregistration.h"

TDK_2DFeatureDetection::TDK_2DFeatureDetection()
{
}

TDK_2DFeatureDetection::~TDK_2DFeatureDetection()
{

}

/*!
 * \brief TDK_2DFeatureDetection::setInputPointCloud
 * \param inPointCloudPtr pointer for train point cloud
 *
 * Function for setting train point cloud that we will use for feature detection
 */
void TDK_2DFeatureDetection::setInputPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inPointCloudPtr)
{
    mv_TrainPointCloudPtr = inPointCloudPtr;
}

/*!
 * \brief TDK_2DFeatureDetection::setInputPointCloud
 * \param inPointCloud reference for train point cloud
 *
 * Function for setting train point cloud that we will use for feature detection
 */

void TDK_2DFeatureDetection::setInputPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &inPointCloud)
{
    mv_TrainPointCloudPtr= boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(inPointCloud);
}

/*!
 * \brief TDK_2DFeatureDetection::getMatchedFeatures
 * \param queryPc pointer for a query point cloud to find mathced features
 * \param outKeyPointsTrain pointer for a point cloud of features from train point cloud
 * \param outKeyPointsQuery pointer for a point cloud of features from query point cloud
 *
 * Method finds and returns feature matches for query and train point clouds
 */

void TDK_2DFeatureDetection::getMatchedFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &queryPc,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr &outKeyPointsTrain,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr &outKeyPointsQuery)
{

    cv::Mat trainImg, queryImg;
    cv::Mat_<CameraSpacePoint> trainImgCameraSpaceMap, queryImgCameraSpaceMap;
    ColorSpacePoint maxTrainImgColorCoords,
                    minTrainImgColorCoords,
                    maxQueryImgColorCoords,
                    minQueryImgColorCoords;

    getIntensityImage(mv_TrainPointCloudPtr,
                      trainImg,
                      trainImgCameraSpaceMap);
    getIntensityImage(queryPc,
                      queryImg,
                      queryImgCameraSpaceMap);

    getImageBoundaries(mv_TrainPointCloudPtr, maxTrainImgColorCoords, minTrainImgColorCoords);
    getImageBoundaries(queryPc, maxQueryImgColorCoords, minQueryImgColorCoords);

    int maxVerticalShiftPxls = INT_MAX;
    //calculating maximum possible vertical shift, images should be approximately on the same level

    if ((abs(maxTrainImgColorCoords.Y - maxQueryImgColorCoords.Y) / maxTrainImgColorCoords.Y) < maxVerticalShift)
    {
        int trainImgHeight = maxTrainImgColorCoords.Y - minTrainImgColorCoords.Y;
        int queryImgHeight = maxQueryImgColorCoords.Y - minQueryImgColorCoords.Y;

        maxVerticalShiftPxls = round(maxVerticalShift * std::max(trainImgHeight, queryImgHeight));
    }

    std::vector<cv::DMatch> matchesRobust,
                            matches;
    std::vector<cv::KeyPoint> keyPtsRobustTrain,
                              keyPtsRobustQuery,
                              keyPtsTrain,
                              keyPtsQuery;
    outKeyPointsTrain->clear();
    outKeyPointsQuery->clear();

    matchFeatures(trainImg,
                  keyPtsRobustTrain,
                  queryImg,
                  keyPtsRobustQuery,
                  matchesRobust,
                  maxVerticalShiftPxls);

    showMatchedFeatures2D(trainImg,
                          keyPtsRobustTrain,
                          queryImg,
                          keyPtsRobustQuery,
                          matchesRobust);
    //we should have at least 4 matches in order to calculate transformation matrix
    if (matchesRobust.size() >= 4)
    {
        qDebug() << "Robust matches number " << matchesRobust.size();
        for(auto it = matchesRobust.begin(); it != matchesRobust.end(); it++)
        {
            cv::DMatch tempMatch = *it;
            //Do not be confused with naming, it is vice versa
            //The interface and language is not consistent in opencv matcher.
            cv::KeyPoint tempKeyPointTrain = keyPtsRobustTrain[tempMatch.queryIdx];
            cv::KeyPoint tempKeyPointQuery = keyPtsRobustQuery[tempMatch.trainIdx];
//            cv::KeyPoint tempKeyPointTrain = keyPtsRobustTrain[tempMatch.trainIdx];
//            cv::KeyPoint tempKeyPointQuery = keyPtsRobustQuery[tempMatch.queryIdx];

            CameraSpacePoint tempSpacePointTrain = trainImgCameraSpaceMap.at<CameraSpacePoint>(
                        static_cast<int>(tempKeyPointTrain.pt.y),
                        static_cast<int>(tempKeyPointTrain.pt.x)
                        );

            CameraSpacePoint tempSpacePointTarget = queryImgCameraSpaceMap.at<CameraSpacePoint>(
                        static_cast<int>(tempKeyPointQuery.pt.y),
                        static_cast<int>(tempKeyPointQuery.pt.x)
                        );

            pcl::PointXYZ tempPointXYZIn(tempSpacePointTrain.X, tempSpacePointTrain.Y, tempSpacePointTrain.Z);
            pcl::PointXYZ tempPointXYZTarget(tempSpacePointTarget.X, tempSpacePointTarget.Y, tempSpacePointTarget.Z);
            outKeyPointsTrain->push_back(tempPointXYZIn);
            outKeyPointsQuery->push_back(tempPointXYZTarget);
        }
    }
    else
    {
        qDebug() << "Robust match method failed, start regular match." << matchesRobust.size();
        matchFeatures(trainImg, keyPtsTrain, queryImg, keyPtsQuery, matches);

        //we should have at least 4 matches in order to calculate transformation matrix
        if (matches.size() >= 4)
        {
            qDebug() << "Regular matches number " << matches.size();
            for(auto it = matches.begin(); it != matches.end(); it++)
            {
                cv::DMatch tempMatch = *it;
                //The interface and language is not consistent in opencv matcher.
                //Beware of the naming
                cv::KeyPoint tempKeyPointTrain = keyPtsTrain[tempMatch.queryIdx];
                cv::KeyPoint tempKeyPointQuery = keyPtsQuery[tempMatch.trainIdx];


                CameraSpacePoint tempSpacePointTrain = trainImgCameraSpaceMap.at<CameraSpacePoint>(
                            static_cast<int>(tempKeyPointTrain.pt.y),
                            static_cast<int>(tempKeyPointTrain.pt.x)
                            );

                CameraSpacePoint tempSpacePointTarget = queryImgCameraSpaceMap.at<CameraSpacePoint>(
                            static_cast<int>(tempKeyPointQuery.pt.y),
                            static_cast<int>(tempKeyPointQuery.pt.x)
                            );

                pcl::PointXYZ tempPointXYZIn(tempSpacePointTrain.X, tempSpacePointTrain.Y, tempSpacePointTrain.Z);
                pcl::PointXYZ tempPointXYZTarget(tempSpacePointTarget.X, tempSpacePointTarget.Y, tempSpacePointTarget.Z);
                outKeyPointsTrain->push_back(tempPointXYZIn);
                outKeyPointsQuery->push_back(tempPointXYZTarget);
            }
        }
        else
        {
            throw std::runtime_error("Found no matches, can not proceed.");
        }
    }
}

/*!
 * \brief TDK_2DFeatureDetection::showKeyPoints
 * \param rgbImg input image
 * \param keyPts input vector of key points
 *
 * Method shows keypoint of image
 */
void TDK_2DFeatureDetection::showKeyPoints(const cv::Mat &rgbImg,
                                           const std::vector<cv::KeyPoint> &keyPts) {
    //Draw keypoints
    cv::Mat img_keyPts;
    cv::drawKeypoints(rgbImg, keyPts, img_keyPts, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    //Show the keypoints
    cv::imshow("Keypoints", img_keyPts);
    cv::waitKey(0);
}

/*!
 * \brief TDK_2DFeatureDetection::showMatchedFeatures2D
 * \param rgb_1 input image
 * \param keyPts_1 input vector of key points
 * \param rgb_2 input image
 * \param keyPts_2 input vector of key points
 * \param matches input vector of matches
 *
 * Method shows matches of two images
 */
void TDK_2DFeatureDetection::showMatchedFeatures2D(const cv::Mat &rgb_1,
                                                   const std::vector<cv::KeyPoint> &keyPts_1,
                                                   const cv::Mat &rgb_2,
                                                   const std::vector<cv::KeyPoint> &keyPts_2,
                                                   const std::vector<cv::DMatch> &matches)
{
    //Show detected matches
    cv::Mat img_matches;
    cv::drawMatches(rgb_1,
                    keyPts_1,
                    rgb_2,
                    keyPts_2,
                    matches,
                    img_matches,
                    cv::Scalar::all(-1),
                    cv::Scalar::all(-1),
                    cv::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    //Show detected matches
    cv::imshow("Feature Matches", img_matches);
    cv::waitKey(0);
}

/*!
 * \brief TDK_2DFeatureDetection::showMatchedFeatures3D
 * \param queryPC pointer for query point cloud
 * \param trainKeyPoints pointer for matched key points point cloud of train point cloud
 * \param queryKeyPoints pointer for matched key points point cloud of query point cloud
 *
 * Method shows matched key points in 3D
 */
void TDK_2DFeatureDetection::showMatchedFeatures3D(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &queryPC,
                                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr &trainKeyPoints,
                                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr &queryKeyPoints)
{
    // Create a more complicate 3d point clud visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbViewer = rgbVis(mv_TrainPointCloudPtr);
    // Add another cloud
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_2(queryPC);
    rgbViewer->addPointCloud<pcl::PointXYZRGB> (queryPC, rgb_2);

    // Visualize the features
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_1(trainKeyPoints, 255, 0, 0);
    rgbViewer->addPointCloud<pcl::PointXYZ> (trainKeyPoints, single_color_1, "feat_1");
    rgbViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "feat_1");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_2(queryKeyPoints, 0, 255, 0);
    rgbViewer->addPointCloud<pcl::PointXYZ> (queryKeyPoints, single_color_2, "feat_2");
    rgbViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "feat_2");
    while (!rgbViewer->wasStopped ())
     {
       rgbViewer->spinOnce (100);
       boost::this_thread::sleep (boost::posix_time::microseconds (100000));
     }
}

/*!
 * \brief TDK_2DFeatureDetection::getIntensityImage
 * \param inPointCloud point cloud used to find intensity image from
 * \param outIntensityImage found intensity image
 * \param outCameraSpaceMap maps corrdinates from point cloud to pixels from output intensity image
 *
 * Function generates intensity image from the input point cloud
 */
void TDK_2DFeatureDetection::getIntensityImage(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inPointCloud,
                                               cv::Mat &outIntensityImage,
                                               cv::Mat_<CameraSpacePoint> &outCameraSpaceMap)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudXYZI(new pcl::PointCloud<pcl::PointXYZI>);

    tdk_PointCloudXYZRGBtoXYZI(inPointCloud, pointCloudXYZI);

    //We will work with depth image rgb image ovelaping only, so we will take into consideration only
    //size of depth image, since it is smaller that the one of rgb image
    outIntensityImage = cv::Mat::zeros(mv_Kinect2Grabber.getDepthWidth(),
                                       mv_Kinect2Grabber.getDepthHeight(),
                                       CV_8UC1);

    outCameraSpaceMap = cv::Mat_<CameraSpacePoint>(mv_Kinect2Grabber.getDepthWidth(),
                                             mv_Kinect2Grabber.getDepthHeight());

    for (auto it = pointCloudXYZI->begin(); it != pointCloudXYZI->end(); it++)
    {
        pcl::PointXYZI tempPointXYZI = *it;
        ColorSpacePoint tempColorCoords;
        CameraSpacePoint tempCameraCoords;

        mv_Kinect2Grabber.convertCameraPointToColorPoint(tempPointXYZI, tempColorCoords);

        tempCameraCoords.X = tempPointXYZI.x;
        tempCameraCoords.Y = tempPointXYZI.y;
        tempCameraCoords.Z = tempPointXYZI.z;

        if ((tempColorCoords.X >= 0 && tempColorCoords.X <= mv_Kinect2Grabber.getDepthWidth()) &&
            (tempColorCoords.Y >= 0 && tempColorCoords.Y <= mv_Kinect2Grabber.getDepthHeight()))
        {
            outIntensityImage.at<uchar>(tempColorCoords.Y, tempColorCoords.X) =
                    static_cast<uchar>(tempPointXYZI.intensity);

            outCameraSpaceMap.at<CameraSpacePoint>(tempColorCoords.Y, tempColorCoords.X) =
                    tempCameraCoords;
        }
    }
}

/*!
 * \brief TDK_2DFeatureDetection::detectFeatures
 * \param img image used for feature detection
 * \param keyPts output array of found keypoints
 *
 * Methods detects features of provided image
 */
void TDK_2DFeatureDetection::detectFeatures(const cv::Mat &img,
                                            std::vector<cv::KeyPoint> &keyPts)
{
    qDebug() << "Feature detection started.";

    //Covert color image to gray
    cv::Mat grayImg;
    if (img.channels() != 1)
    {
        cv::cvtColor(img, grayImg, cv::COLOR_RGB2GRAY);
    }
    else
    {
        grayImg = img;
    }

    //Detect SIFT features
    cv::SiftFeatureDetector siftDetector(100, 5);
    siftDetector.detect(grayImg, keyPts);
    qDebug() << "Feature dection ended, number of detected key points: " << keyPts.size();
}

/*!
 * \brief TDK_2DFeatureDetection::matchFeatures
 * \param rgb_1 input image for feature matching
 * \param keyPts_1 output vector of keypoints
 * \param rgb_2 input image for feature matching
 * \param keyPts_2 output vector of keypoints
 * \param matches output vector of found matches
 * \param robustMatch parameter specifies whether method should remove outliers
 *
 * Finds matching features of two input images, can remove outliers if latter needed
 */

void TDK_2DFeatureDetection::matchFeatures(const cv::Mat &rgb_1,
                                           std::vector<cv::KeyPoint> &keyPts_1,
                                           const cv::Mat &rgb_2,
                                           std::vector<cv::KeyPoint> &keyPts_2,
                                           std::vector<cv::DMatch> &matches,
                                           const int maxVertShiftPxls)
{
    qDebug() << "Feature matching started.";
    //Detect SIFT features
    cv::Mat gray_1;
    cv::Mat gray_2;

    if (rgb_1.channels() != 1)
    {
        cv::cvtColor(rgb_1, gray_1, cv::COLOR_RGB2GRAY);
    }
    else
    {
        gray_1 = rgb_1;
    }

    if (rgb_2.channels() != 1)
    {
        cv::cvtColor(rgb_2, gray_2, cv::COLOR_RGB2GRAY);
    }
    else
    {
        gray_2 = rgb_2;
    }

    this->detectFeatures(gray_1, keyPts_1);
    this->detectFeatures(gray_2, keyPts_2);

    //Compute descriptor
    cv::SiftDescriptorExtractor siftDesExtractor;
    cv::Mat descriptors_1,
            descriptors_2;
    siftDesExtractor.compute(gray_1, keyPts_1, descriptors_1);
    siftDesExtractor.compute(gray_2, keyPts_2, descriptors_2);
    qDebug() << "Descriptors calculated, descriptors type: " << descriptors_1.type();

    //-- Feature matching using descriptors
    //cv::FlannBasedMatcher matcher;
    cv::BFMatcher matcher(cv::NORM_L2, true);
    matcher.match(descriptors_1, descriptors_2, matches);

    // SIFT descriptor for CV_32F has type 5
    qDebug() << "Matched descriptors: " << matches.size();

    //check whether vertical shift limit was given
    if (maxVertShiftPxls != INT_MAX)
    {
        qDebug() << "Vertical outliers removal is started." << "Max vertical distance = " << maxVertShiftPxls;

        // Only consider matches with small vertical distances
        std::vector<cv::DMatch> closestMatches;

        std::copy_if(matches.begin(), matches.end(),
                std::back_insert_iterator<std::vector<cv::DMatch>>(closestMatches),
                [&] (cv::DMatch tempMatch) {
                        //The interface and language is not consistent in opencv matcher.
                        //Beware of the naming
//                        cv::KeyPoint tempKeyPoint1 = keyPts_1[tempMatch.queryIdx];
//                        cv::KeyPoint tempKeyPoint2 = keyPts_2[tempMatch.trainIdx];
                          cv::KeyPoint tempKeyPointTrain = keyPts_1[tempMatch.trainIdx];
                          cv::KeyPoint tempKeyPointQuery = keyPts_2[tempMatch.queryIdx];

                        //if (abs(tempKeyPoint1.pt.y - tempKeyPoint2.pt.y) <= maxVertShiftPxls)
                        if (abs(tempKeyPointTrain.pt.y - tempKeyPointQuery.pt.y) <= maxVertShiftPxls)
                        {
                            return true;
                        }
                        else
                        {
                            return false;
                        }
            });

        matches.clear();

        std::copy(closestMatches.begin(),
                  closestMatches.end(),
                  std::back_insert_iterator<std::vector<cv::DMatch>>(matches));

        qDebug() << "Good matches number: " << matches.size();
    }


    std::sort(matches.begin(), matches.end(), [] (cv::DMatch elem1, cv::DMatch elem2) {
        return elem1.distance < elem2.distance;

    });

    qDebug() << "Feature matching is completed.";
}

/*!
 * \brief TDK_2DFeatureDetection::rgbVis
 * \param cloud input point cloud to be shown
 * \return 3D viewer
 *
 * Method opens 3D viewer and adds point cloud to it
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> TDK_2DFeatureDetection::rgbVis (
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        const std::string pointCloudId)
{

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, pointCloudId);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, pointCloudId);
  viewer->addCoordinateSystem (0.10);
  viewer->initCameraParameters ();
  return (viewer);
}

/*!
 * \brief TDK_2DFeatureDetection::getImageBoundaries
 * \param inPointCloud input point cloud
 * \param maxColorCoords output max values of pixels' coordinates for X, Y axes
 * \param minColorCoords output min values of pixels' coordinates for X, Y axes
 *
 * Method returns max and min values of pixels' coordinates for a given point cloud
 */
void TDK_2DFeatureDetection::getImageBoundaries(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inPointCloud,
        ColorSpacePoint &maxColorCoords,
        ColorSpacePoint &minColorCoords)
{
    maxColorCoords.X = 0;
    maxColorCoords.Y = 0;
    minColorCoords.X = INT_MAX;
    minColorCoords.Y = INT_MAX;

    qDebug() << "Looking for image boundaries...";

    for (auto it = inPointCloud->begin(); it != inPointCloud->end(); it++)
    {
        pcl::PointXYZRGB tempPointXYZRGB = *it;
        ColorSpacePoint tempColorCoords;

        mv_Kinect2Grabber.convertCameraPointToColorPoint(tempPointXYZRGB, tempColorCoords);

        if ((tempColorCoords.X >= 0 && tempColorCoords.X <= mv_Kinect2Grabber.getDepthWidth()) &&
            (tempColorCoords.Y >= 0 && tempColorCoords.Y <= mv_Kinect2Grabber.getDepthHeight()))
        {
            maxColorCoords.X = std::max(maxColorCoords.X, tempColorCoords.X);
            maxColorCoords.Y = std::max(maxColorCoords.Y, tempColorCoords.Y);
            minColorCoords.X = std::min(minColorCoords.X, tempColorCoords.X);
            minColorCoords.Y = std::min(minColorCoords.Y, tempColorCoords.Y);
        }
    }
}
