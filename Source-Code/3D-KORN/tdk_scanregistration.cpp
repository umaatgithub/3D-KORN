#include "tdk_scanregistration.h"
#include "tdk_filters.h"

#include <QDebug>

#include <iostream>

#include <algorithm>


using namespace std;

TDK_ScanRegistration::TDK_ScanRegistration()
{
    //Empty constructor
    mv_registerInRealTime = false;
    this->setDefaultParameters();
}

/////////////////////////////////////////////////////

TDK_ScanRegistration::TDK_ScanRegistration(const bool registerInRealTime)
{
    //Empty constructor
    mv_registerInRealTime = registerInRealTime;
    this->setDefaultParameters();
}

/////////////////////////////////////////////////////

TDK_ScanRegistration::TDK_ScanRegistration(
        const pcl::PointWithViewpoint scannerCenter,
        const bool registerInRealTime)
{
    mv_registerInRealTime = registerInRealTime;
    this->setDefaultParameters();
    this->setScannerRotationAxis(scannerCenter);
}

/////////////////////////////////////////////////////
void TDK_ScanRegistration::setDefaultParameters()
{
    mv_scannerCenterRotationSet = false;
    mv_accumulatedRotation = 0.0;

    //Size in meters used in the downsampling of input for inital alignment
    mv_voxelSideLength = 0.015;

    //Normal radius search for correspondence matching
    mv_normalRadiusSearch=0.05;

    //Max distance in meters between two points to find correspondances for SVD
    mv_SVD_MaxDistance=0.15;

    //Max distance in meters between two points to find correspondances for ICP
    mv_ICP_MaxCorrespondenceDistance = 0.05;

    //Max distance in meters between two points to find correspondances for
    //loop closing and IncrementalICP layers
    mv_ICPPost_MaxCorrespondanceDistance = 0.03;
}

/////////////////////////////////////////////////////
TDK_ScanRegistration::~TDK_ScanRegistration()
{
    //Destructor is empty since all of the dynamic allocation
    //is performed with boost smart pointers
}

/////////////////////////////////////////////////////
bool TDK_ScanRegistration::addNextPointCloud(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointcloud,
        const float degreesRotatedY)
{
    //Prepare arrays for use later when we want to register everything at once
    if(!mv_registerInRealTime){
        mv_originalPCs.push_back(inputPointcloud);
        mv_originalPointcloudsYRotation.push_back(degreesRotatedY);
        return true;
    }

    //If we want to register in realtime
    /*if(mv_scannerCenterRotationSet){
        qDebug() << "ScanRegistration: Rotating with " << degreesRotatedY << "º ";

        //Transform pointcloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedInputPointcloud(new  pcl::PointCloud<pcl::PointXYZRGB>());

        //Create transform matrix that compensates for turning table orientation and distance, and rotation
        Eigen::Transform<float,3,Eigen::Affine> transform =
                Eigen::AngleAxisf(mv_scannerCenter.vp_z*(M_PI/180.0), Eigen::Vector3f::UnitZ()) *
                Eigen::AngleAxisf(mv_accumulatedRotation*(M_PI/180.0), Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(mv_scannerCenter.vp_x*(M_PI/180.0), Eigen::Vector3f::UnitX())* //20 for pamir
                Eigen::Translation3f(-mv_scannerCenter.x, -mv_scannerCenter.y, -mv_scannerCenter.z);


        pcl::transformPointCloud(*inputPointcloud, *transformedInputPointcloud, transform.matrix());
        mv_accumulatedRotation += degreesRotatedY;

        //Add reference of original pointcloud to array
        mv_originalRotatedPCs.push_back(transformedInputPointcloud);

        //Remove outliers and store reference to denoised Pointcloud
        int neighs = 80;
        float stddev = 2.5;
        mv_originalRotatedDenoisedPCs.push_back(
                    mf_outlierRemovalPC(
                        mf_outlierRemovalPC(
                            mf_outlierRemovalPC(
                                mv_originalRotatedPCs.back(),
                                neighs, stddev)
                            , neighs, stddev),
                        neighs, stddev)
                    );

        //Call process that will roughly align the last pointcloud to all previous ones
        mf_processCorrespondencesSVDICP();

        return true;
    }*/
    else{
        qDebug() << "ScanRegistration: Add pc w/out Compensation or Prealignment";
        mv_originalRotatedPCs.push_back(inputPointcloud);

        //Remove outliers and store reference to denoised Pointcloud
        mv_originalRotatedDenoisedPCs.push_back(mf_outlierRemovalPC(mv_originalRotatedPCs.back()));

        mf_processInPostWithICP();

        return true;
    }
}


/////////////////////////////////////////////////////
bool TDK_ScanRegistration::addAllPointClouds(
        const vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &inputPCs,
        const vector<float> degreesRotatedY
        )
{

    for(int i = 0; i < inputPCs.size(); i++){
        if(!addNextPointCloud(inputPCs[i], degreesRotatedY[i])){
            return false;
        }
    }

    return true;
}


/////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr TDK_ScanRegistration::getLastDownSampledPointcloud()
{
    return mv_downSampledPCs.back();
}

/////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZRGB>::Ptr TDK_ScanRegistration::getRoughlyAlignedPC()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedAlignedOriginal(new pcl::PointCloud<pcl::PointXYZRGB>());

    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator it;
    for (it = mv_alignedOriginalPCs.begin(); it != mv_alignedOriginalPCs.end(); ++it)
        *mergedAlignedOriginal += *(*it);

    return mf_outlierRemovalPC(mergedAlignedOriginal, 1.0);
}





pcl::PointCloud<pcl::PointXYZRGB>::Ptr TDK_ScanRegistration::Register(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &Data ) {

    // PCL_INFO (" \n Loaded %d datasets ... \n", (int)Data.size ());

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fusedCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    const float VoxelGridLeafSize = 0.002; // 0.004


    result2 = Data[0];
    std::cout<< "data size"<< Data.size ()<<endl;
    for (size_t i = 1; i < Data.size (); ++i)
    {

        cloud_src = result2; // source
        cloud_tgt = Data[i]; // target

        TDK_Filters::mf_FilterVoxelGridDownsample (cloud_src, src, VoxelGridLeafSize);
        TDK_Filters::mf_FilterVoxelGridDownsample (cloud_tgt, tgt, VoxelGridLeafSize);


        if (mv_use2DFeatureDetection == true){
            // DO feature detection and run ICP/ICP Normal on the point clouds

            pcl::PointCloud<pcl::PointXYZ>::Ptr trainKeyPoints(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr queryKeyPoints(new pcl::PointCloud<pcl::PointXYZ>());

            TDK_2DFeatureDetection mv_2DFeatureDetectionPtr;
            mv_2DFeatureDetectionPtr.setInputPointCloud(src);
            mv_2DFeatureDetectionPtr.getMatchedFeatures(tgt, trainKeyPoints, queryKeyPoints);

            std::cout << "FEATURE MATCHING " << std::endl;
           // mv_2DFeatureDetectionPtr.showMatchedFeatures3D(tgt, trainKeyPoints, queryKeyPoints);
            MatchRegistration(src, tgt, trainKeyPoints, queryKeyPoints, fusedCloud);

            std::cout << "MATCHING DONE! " << std::endl;


             //To visualize point clouds in a different window
            boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbViewer2 = rgbVis(fusedCloud);
                        while (!rgbViewer2->wasStopped ())
                        {
                            rgbViewer2->spinOnce (100);
                            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
                        }

            *fusedCloud += *cloud_tgt;

           //result2 =  TDK_ScanRegistration::mf_outlierRemovalPC(fusedCloud);
            TDK_Filters::mf_FilterStatisticalOutlierRemoval (fusedCloud , result2);
            //TDK_Filters::mf_FilterPassthroughBri(fusedCloud, result2);

            std::cout << "ICP between frame " << i << " and " << i+1 << std::endl;
        }
        else{

            if (mv_ICP_Normals == false)
                result1 = TDK_ScanRegistration::ICP(src, tgt);
            else
                result1 = TDK_ScanRegistration::ICPNormal(src, tgt);

            *result1 += *cloud_tgt;
            result2 = result1;
            //result2 =  TDK_ScanRegistration::mf_outlierRemovalPC(result1);


        }
    }


    return result2;

}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr TDK_ScanRegistration::ICPNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt){


    float MaxDistance=0.015;
    float RansacVar = 0.01;
    float Iterations = 100;


    pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt (new pcl::PointCloud<pcl::PointNormal>);

    pcl::PointCloud<pcl::PointNormal>::Ptr normals_icp (new pcl::PointCloud<pcl::PointNormal>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_norm (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> norm_est;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

    qDebug() << "Tree and Normal estimation set up!";
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (12);

    norm_est.setInputCloud (src);
    norm_est.compute (*points_with_normals_src);
    qDebug() << "Normals computed! (1)";
    pcl::copyPointCloud (*src, *points_with_normals_src);

    norm_est.setInputCloud (tgt);
    norm_est.compute (*points_with_normals_tgt);
    qDebug() << "Normals computed! (2)";
    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> reg;
    reg.setTransformationEpsilon (1e-8);

    reg.setMaxCorrespondenceDistance (MaxDistance);
    reg.setRANSACOutlierRejectionThreshold (RansacVar); // 0.05
    reg.setMaximumIterations (Iterations);


    reg.setInputSource (points_with_normals_src);
    reg.setInputTarget (points_with_normals_tgt);
    reg.align (*normals_icp);

    qDebug() << "Normals aligned!";

    std::cout << "Normals converged with score: " << reg.getFitnessScore() << std::endl;

    Eigen::Matrix4f transform_normals = reg.getFinalTransformation ();
    qDebug() << "Transformations obtained!";
    pcl::transformPointCloud (*src, *cloud_norm, transform_normals);

    std::cout<<cloud_norm<<std::endl;
    return cloud_norm;

}


///////////////////////////////////////////////////

pcl::PointCloud<pcl::PointXYZRGB>::Ptr TDK_ScanRegistration::ICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt){
    // Start first ICP
    float MaxDistance=0.015;
    float RansacVar = 0.01;
    float Iterations = 100;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaxCorrespondenceDistance (MaxDistance); //0.10 //0.015
    icp.setRANSACOutlierRejectionThreshold (RansacVar); // 0.05
    icp.setTransformationEpsilon (1e-8);
    icp.setMaximumIterations (Iterations);
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.align(*Final);

    std::cout << "ICP converged with score: " << icp.getFitnessScore() << std::endl;

    return Final;

}

/////////////////////////////////////////////////////

pcl::PointCloud<pcl::PointXYZRGB>::Ptr TDK_ScanRegistration::Process_and_getAlignedPC()
{
    if(! mv_registerInRealTime){
        mv_registerInRealTime = true;
        addAllPointClouds(mv_originalPCs, mv_originalPointcloudsYRotation);
    }

    if(!mv_scannerCenterRotationSet){
        //qDebug() << "ScanRegistration: PostProcessing without prealignment.";
        mv_ICPPost_MaxCorrespondanceDistance = 0.15;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedAlignedOriginal(new pcl::PointCloud<pcl::PointXYZRGB>());


        qDebug() << "ScanRegistration: PostProcessing with feature detection";

        emit mf_SignalStatusChanged(tr("Registration started..."), QColor(Qt::red));

        mergedAlignedOriginal = TDK_ScanRegistration::Register(mv_alignedOriginalPCs);

        emit mf_SignalStatusChanged(tr("Registration done!"), QColor(Qt::darkGreen));


    return mergedAlignedOriginal;
}

/////////////////////////////////////////////////////

vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* TDK_ScanRegistration::getRoughlyAlignedPCs()
{
    return &mv_alignedOriginalPCs;
}


/////////////////////////////////////////////////////

vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* TDK_ScanRegistration::getRotationCompensatedPCs()
{
    return &mv_originalRotatedDenoisedPCs;
}

/////////////////////////////////////////////////////

//bool TDK_ScanRegistration::mf_processCorrespondencesSVDICP()
//{

//    //Downsample and add pointcloud to array
//    mv_downSampledPCs.push_back(
//                mf_voxelDownSamplePointCloud(
//                    mv_originalRotatedDenoisedPCs.back(), mv_voxelSideLength
//                    )
//                );

//    //Compute normals from downsampled pointcloud for correspondence estimation
//    mv_downSampledNormals.push_back(
//                mf_computeNormals(
//                    mv_downSampledPCs.back(), mv_normalRadiusSearch
//                    )
//                );

//    //If pointcloud array has been initialized
//    if(mv_originalRotatedPCs.size() > 1){
//        //Compute correspondences between new pointcloud and last aligned pointcloud
//        mv_downsampledCorrespondences.push_back(
//                    mf_estimateCorrespondences(
//                        mv_downSampledPCs.back(),     *(mv_downSampledPCs.end()-2),
//                        mv_downSampledNormals.back(), *(mv_downSampledNormals.end()-2),
//                        mv_SVD_MaxDistance
//                        )
//                    );


//        //Align downsampled pointclouds using SVD and get transform to apply on original later
//        Eigen::Matrix4f SVDtransform;
//        mv_alignedDownSampledPCs.push_back(
//                    mf_SVDInitialAlignment(
//                        mv_downSampledPCs.back(),        //New DownsampledPointclout to be aligned
//                        mv_alignedDownSampledPCs.back(), //Previous DownsampledAlignedPointcloud
//                        mv_downsampledCorrespondences.back(), SVDtransform
//                        )
//                    );

//        //Store rough alignment transform for use later
//        mv_transformationMatrices.push_back(SVDtransform);

//        //Perform second step of initial alignment with ICP
//        Eigen::Matrix4f ICPtransform;
//        mv_alignedDownSampledPCs.back() =
//                mf_iterativeClosestPointFinalAlignment<pcl::PointXYZ>(
//                    *(mv_alignedDownSampledPCs.end()-1),
//                    *(mv_alignedDownSampledPCs.end()-2),
//                    mv_ICP_MaxCorrespondenceDistance, ICPtransform
//                    );

//        //Add both transforms together to make initial alignment transform
//        mv_transformationMatrices.back() = ICPtransform * mv_transformationMatrices.back();

//        //Apply compound transformation to roughly align original pointcloud to previous one
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr alignedOriginal(new pcl::PointCloud<pcl::PointXYZRGB>());
//        pcl::transformPointCloud(*mv_originalRotatedDenoisedPCs.back(), *alignedOriginal, mv_transformationMatrices.back());
//        mv_alignedOriginalPCs.push_back(alignedOriginal);

//        //Recompute Normals once the Downsampled pointcloud has been aligned for use later
//        mv_downSampledNormals.back() =
//                mf_computeNormals(
//                    mv_alignedDownSampledPCs.back(),
//                    mv_normalRadiusSearch
//                    );
//    }else{
//        //If its the first pointcloud, no need to align anything (it is already aligned to itself)
//        mv_alignedOriginalPCs.push_back(mv_originalRotatedDenoisedPCs.back());
//        mv_alignedDownSampledPCs.push_back(mv_downSampledPCs.back());
//    }

//    return true;
//}


/////////////////////////////////////////////////////

bool TDK_ScanRegistration::mf_processInPostWithICP()
{
    qDebug() << "ScanRegistration: Process pc w/out Compensation or Prealignment";
    mv_alignedOriginalPCs.push_back(mv_originalRotatedDenoisedPCs.back());

    return true;
}

/////////////////////////////////////////////////////

pcl::PointCloud<pcl::PointXYZRGB>::Ptr TDK_ScanRegistration::mf_voxelDownSamplePointCloud(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
        const float &voxelSideLength
        )
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downSampledPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);


    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setLeafSize (voxelSideLength, voxelSideLength, voxelSideLength);

    vg.setInputCloud (cloud_in);
    vg.filter (*downSampledPointCloud);

    return downSampledPointCloud;
}

/////////////////////////////////////////////////////

pcl::PointCloud<pcl::PointXYZ>::Ptr TDK_ScanRegistration::mf_voxelDownSamplePointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
        const float &voxelSideLength
        )
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr downSampledPointCloud(new pcl::PointCloud<pcl::PointXYZ>);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    //PointCloudXYZRGBtoXYZ(cloud_in, cloud_in_xyz);

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize (voxelSideLength, voxelSideLength, voxelSideLength);

    vg.setInputCloud (cloud_in);
    vg.filter (*downSampledPointCloud);

    return downSampledPointCloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
TDK_ScanRegistration::mf_outlierRemovalPC(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
        const float meanK,
        const float std_dev
        )
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter (true);
    sorfilter.setInputCloud (cloud_in);
    sorfilter.setMeanK (meanK);
    sorfilter.setStddevMulThresh (std_dev);
    sorfilter.filter (*cloud_out);

    return cloud_out;
}

/////////////////////////////////////////////////////

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
TDK_ScanRegistration::mf_outlierRemovalPC(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
        const float meanK,
        const float std_dev
        )
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sorfilter (true);
    sorfilter.setInputCloud (cloud_in);
    sorfilter.setMeanK (meanK);
    sorfilter.setStddevMulThresh (std_dev);
    sorfilter.filter (*cloud_out);

    return cloud_out;
}

/////////////////////////////////////////////////////

void
TDK_ScanRegistration::setScannerRotationAxis(const pcl::PointWithViewpoint &value)
{
    mv_scannerCenterRotationSet = true;
    mv_scannerCenter = value;

    qDebug() << "ScanRegistration: Center of rotation = ("
             << value.x << ", "
             << value.y << ", "
             << value.z << ", "
             << value.vp_x << ", "
             << value.vp_y << ", "
             << value.vp_z << ")";
}

pcl::PointWithViewpoint TDK_ScanRegistration::getScannerRotationAxis() const
{
    return mv_scannerCenter;
}

float TDK_ScanRegistration::get_normalRadiusSearch() const
{
    return mv_normalRadiusSearch;
}

void TDK_ScanRegistration::set_normalRadiusSearch(float value)
{
    mv_normalRadiusSearch = value;
}

float TDK_ScanRegistration::get_voxelSideLength() const
{
    return mv_voxelSideLength;
}

void TDK_ScanRegistration::set_PostICP_MaxCorrespondanceDistance(float value)
{
    mv_ICPPost_MaxCorrespondanceDistance = value;
}

bool TDK_ScanRegistration::getRegisterInRealTime() const
{
    return mv_registerInRealTime;
}

void TDK_ScanRegistration::setRegisterInRealTime(bool value)
{
    mv_registerInRealTime = value;
}

void TDK_ScanRegistration::set_voxelSideLength(float value)
{
    mv_voxelSideLength = value;
}

double TDK_ScanRegistration::get_SVD_MaxDistance() const
{
    return mv_SVD_MaxDistance;
}


float TDK_ScanRegistration::get_ICP_MaxCorrespondenceDistance() const
{
    return mv_ICP_MaxCorrespondenceDistance;
}

float TDK_ScanRegistration::get_ICPPost_MaxCorrespondanceDistance() const
{
    return mv_ICPPost_MaxCorrespondanceDistance;
}

void
TDK_ScanRegistration::set_SVD_MaxDistance(double value)

{
    mv_SVD_MaxDistance = value;
}

void
TDK_ScanRegistration::set_ICP_MaxCorrespondenceDistance(float value)
{
    mv_ICP_MaxCorrespondenceDistance = value;
}


void TDK_ScanRegistration::MatchRegistration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr refCloud,
                                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampleCloud,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr refMatch,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr sampleMatch,
                                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr fusedCloud)
{
    // Define an icp regisration object with default parameter setting
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(sampleMatch);
    icp.setInputTarget(refMatch);
    icp.setRANSACOutlierRejectionThreshold(0.05);
    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align (aligned);
    qDebug() << "Matching of features done";

    // Estimate the rigid transformation
    Eigen::Matrix4f transMat = icp.getFinalTransformation();
    qDebug() << "translation z" << transMat(2, 3);
    qDebug() << "Estimate the rigid transformation";

    // Transform the sample cloud to reference cloud coordinate
    pcl::transformPointCloud(*sampleCloud, *fusedCloud, transMat);
    //To visualize point clouds in a different window

    //    for(int i=0; i<refCloud->size(); i++)
    //    {
    //            fusedCloud->push_back(refCloud->at(i));
    //    }
    //    qDebug() << "transformPointCloud";

    // Registration refinement using ICP with all points

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
        copyColor2XYZ(fusedCloud, cloud_in);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
        copyColor2XYZ(refCloud, cloud_out);
        icp.setInputCloud(cloud_in);
        icp.setInputTarget(cloud_out);
        icp.align(aligned);
        transMat = icp.getFinalTransformation();
        pcl::transformPointCloud(*fusedCloud, *fusedCloud, transMat);
        for(int i=0; i<refCloud->size(); i++)
          {
            fusedCloud->push_back(refCloud->at(i));
          }

//        boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbViewer2 = rgbVis(fusedCloud);
//                    while (!rgbViewer2->wasStopped ())
//                    {
//                        rgbViewer2->spinOnce (100);
//                        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//                    }

//    if (mv_ICP_Normals == false)
//        fusedCloud = TDK_ScanRegistration::ICP(refCloud, fusedCloud);
//    else
//        fusedCloud = TDK_ScanRegistration::ICPNormal(refCloud, fusedCloud);
//        for(int i=0; i<refCloud->size(); i++)
//          {
//            fusedCloud->push_back(refCloud->at(i));
//          }


    qDebug() << "Registration refinement done";
    // Fuse with the reference cloud
    //To visualize point clouds in a different window

}

/////////////////////////////////////////////////////
void PointCloudXYZRGBtoXYZ(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &out
        )
{
    out->empty();
    out->points.resize(in->points.size());
    for (size_t i = 0; i < out->points.size(); i++) {
        out->points[i].x = in->points[i].x;
        out->points[i].y = in->points[i].y;
        out->points[i].z = in->points[i].z;
    }
}


void tdk_PointCloudXYZRGBtoXYZI(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &out
        )
{
    for_each(in->begin(),
             in->end(),
             [&out] (pcl::PointXYZRGB pRGB) {
        pcl::PointXYZI pI{};
        pcl::PointXYZRGBtoXYZI(pRGB, pI);
        out->push_back(pI);
    });
}



// Extract color information
void copyColor2XYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
    for(int i=0; i<cloud_in->size();i++)
    {
        pcl::PointXYZ pt;
        pt.x = cloud_in->at(i).r/255.0;
        pt.y = cloud_in->at(i).g/255.0;
        pt.z = cloud_in->at(i).b/255.0;
        cloud_out->push_back(pt);
    }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reference cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reference cloud");
    viewer->addCoordinateSystem (0.10);
    viewer->initCameraParameters ();
    return (viewer);
}

