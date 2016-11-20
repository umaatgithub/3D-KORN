#include "tdk_scanregistration.h"

#include <QDebug>


using namespace std;

TDK_ScanRegistration::TDK_ScanRegistration()
{
    //Empty constructor
    mv_scannerCenterRotationSet = false;
    mv_accumulatedRotation = 0.0;

    mv_voxelSideLength = 0.02;
    mv_FeatureRadiusSearch=0.05;

    mv_SAC_MinSampleDistance = 0.01;
    mv_SAC_MaxCorrespondenceDistance = 0.3;
    mv_SAC_MaximumIterations = 500;

    mv_ICP_MaxCorrespondenceDistance = 0.06; //0.12

    mv_ISS_resolution = 0.003;  //0.004 ,767 points
    mv_ISS_SalientRadius = 6*mv_ISS_resolution;
    mv_ISS_NonMaxRadius = 4*mv_ISS_resolution;
    mv_ISS_Gamma21 =0.975;
    mv_ISS_Gamma32 =0.975;
    mv_ISS_MinNeighbors = 5; //5
    mv_ISS_Threads = 1;

    mv_SVD_MaxDistance=0.25;
}

bool TDK_ScanRegistration::addNextPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointcloud)
{
    mv_originalPointClouds.push_back(inputPointcloud);

    //Approach 1 SAC ICP
    //    mf_processVoxelSacIcp();
    //mf_processCorrespondencesSVDICP();
    mf_processVoxelIcp();

    return true;
}

bool TDK_ScanRegistration::addNextPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointcloud, float degreesRotatedY)
{
    if(mv_scannerCenterRotationSet){
        //Transform pointcloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedInputPointcloud(new  pcl::PointCloud<pcl::PointXYZRGB>());

        Eigen::Vector3f center_rot(mv_scannerCenterRotation.x, 0.0, mv_scannerCenterRotation.z);
        mv_accumulatedRotation += degreesRotatedY;

        Eigen::Transform<float,3,Eigen::Affine> transform =
                Eigen::AngleAxisf(mv_accumulatedRotation*(M_PI/180.0), Eigen::Vector3f::UnitY()) *
                Eigen::Translation3f(-center_rot);

        pcl::transformPointCloud(*inputPointcloud, *transformedInputPointcloud, transform);

        addNextPointCloud(transformedInputPointcloud);

        return true;
    }else{
        qWarning() << "Scanner Center Rotation NOT SET, To prerotate pcs you need to set it";
        //addNextPointCloud(inputPointcloud);
        return false;
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TDK_ScanRegistration::getLastDownSampledPointcloud()
{
    return mv_downSampledPointClouds.back();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TDK_ScanRegistration::mf_voxelDownSamplePointCloud(const PointCloudT::Ptr &cloud_in, const float &voxelSideLength)
{
    PointCloudXYZ::Ptr downSampledPointCloud(new PointCloudXYZ);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudXYZRGBtoXYZ(cloud_in, cloud_in_xyz);

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize (voxelSideLength, voxelSideLength, voxelSideLength);

    vg.setInputCloud (cloud_in_xyz);
    vg.filter (*downSampledPointCloud);

    return downSampledPointCloud;
}

pcl::PointCloud<pcl::Normal>::Ptr TDK_ScanRegistration::mf_computeNormals(const PointCloudXYZ::Ptr &cloud_in, const float &searchRadius)
{
    SurfaceNormalsT::Ptr normals(new SurfaceNormalsT);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setInputCloud (cloud_in);
    //norm_est.setSearchMethod (new pcl::search::KdTree<pcl::PointXYZRGB>);
    norm_est.setRadiusSearch (searchRadius);
    norm_est.compute (*normals);
    return normals;
}

bool TDK_ScanRegistration::mf_processVoxelSacIcp()
{
    //Downsample pointcloud and push to Downsampled vector
    mv_downSampledPointClouds.push_back(mf_voxelDownSamplePointCloud(mv_originalPointClouds.back(), mv_voxelSideLength));
    mv_downSampledNormals.push_back(mf_computeNormals(mv_downSampledPointClouds.back(),mv_FeatureRadiusSearch));
    mv_downSampledFeatures.push_back(mf_computeLocalFPFH33Features(mv_downSampledPointClouds.back(), mv_downSampledNormals.back(),mv_FeatureRadiusSearch));

    if(mv_originalPointClouds.size() > 1){
        mf_sampleConsensusInitialAlignment();
        mf_iterativeClosestPointFinalAlignment();

        mv_downSampledNormals.back() = mf_computeNormals(mv_alignedDownSampledPointClouds.back(),mv_FeatureRadiusSearch);
        mv_downSampledFeatures.back() = mf_computeLocalFPFH33Features(mv_alignedDownSampledPointClouds.back(), mv_downSampledNormals.back(),mv_FeatureRadiusSearch);
    }else{
        mv_alignedDownSampledPointClouds.push_back(mv_downSampledPointClouds.back());
        mv_alignedOriginalPointClouds.push_back(mv_originalPointClouds.back());
    }
    //qDebug() << "Donwsampled Point Cloud Dimension is " << (--mv_downSampledPointClouds.end())->get()->points.size()  <<endl;
    //qDebug() << "features Dimension is " << mv_downSampledFeatures.back().get()->points.size()  <<endl;

    return true;
}

void TDK_ScanRegistration::setMv_ISS_resolution(double value)
{
    mv_ISS_resolution = value;
}

void TDK_ScanRegistration::setMv_scannerCenterRotation(const pcl::PointXYZ &value)
{
    mv_scannerCenterRotationSet = true;
    mv_scannerCenterRotation = value;
}

void TDK_ScanRegistration::setMv_SVD_MaxDistance(double value)
{
    mv_SVD_MaxDistance = value;
}

void TDK_ScanRegistration::setMv_ICP_MaxCorrespondenceDistance(float value)
{
    mv_ICP_MaxCorrespondenceDistance = value;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr
TDK_ScanRegistration::mf_computeLocalFPFH33Features (const PointCloudXYZ::Ptr &cloud_in, const SurfaceNormalsT::Ptr &normal_in, const float &searchRadius)
{
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features = pcl::PointCloud<pcl::FPFHSignature33>::Ptr (new pcl::PointCloud<pcl::FPFHSignature33>);

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud (cloud_in);
    fpfh_est.setInputNormals (normal_in);
    fpfh_est.setRadiusSearch (searchRadius);
    fpfh_est.compute (*features);

    return features;
}


void TDK_ScanRegistration::mf_sampleConsensusInitialAlignment(){
    //Create initial SAC aligner
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac;

    sac.setMinSampleDistance(mv_SAC_MinSampleDistance);
    sac.setMaxCorrespondenceDistance(mv_SAC_MaxCorrespondenceDistance);
    sac.setMaximumIterations(mv_SAC_MaximumIterations);

    //Cloud to be transformed
    sac.setInputCloud(mv_downSampledPointClouds.back());
    sac.setSourceFeatures(mv_downSampledFeatures.back());

    //Target frame pointcloud
    sac.setInputTarget( mv_alignedDownSampledPointClouds.back() );
    sac.setTargetFeatures( *(mv_downSampledFeatures.end()-2) );

    PointCloudXYZ::Ptr alignedDownSampledSource(new PointCloudXYZ);
    sac.align(*alignedDownSampledSource);

    mv_alignedDownSampledPointClouds.push_back(alignedDownSampledSource);
    mv_transformationMatrixAlignment.push_back(sac.getFinalTransformation());
}

void TDK_ScanRegistration::mf_iterativeClosestPointFinalAlignment(){
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    icp.setMaxCorrespondenceDistance(mv_ICP_MaxCorrespondenceDistance);
    //qDebug() << "Max icp: " << icp.getMaximumIterations();
    icp.setMaximumIterations(100); //originally it was 10

    icp.setInputCloud(mv_alignedDownSampledPointClouds.back());
    icp.setInputTarget(*(mv_alignedDownSampledPointClouds.end()-2));

    PointCloudXYZ::Ptr alignedDownSampledSource(new PointCloudXYZ);
    icp.align(*alignedDownSampledSource);

    qDebug() << "Fitness: " << icp.getFitnessScore();

    mv_alignedDownSampledPointClouds.back() = alignedDownSampledSource;
    mv_transformationMatrixAlignment.back() = icp.getFinalTransformation()*mv_transformationMatrixAlignment.back();

    PointCloudT::Ptr alignedOriginalSource(new PointCloudT);
    pcl::transformPointCloud(*mv_originalPointClouds.back(), *alignedOriginalSource, mv_transformationMatrixAlignment.back());
    mv_alignedOriginalPointClouds.push_back(alignedOriginalSource);
}

vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* TDK_ScanRegistration::mf_getAlignedPointClouds()
{
    return &mv_alignedOriginalPointClouds;
}

vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* TDK_ScanRegistration::mf_getOriginalPointClouds()
{
    return &mv_originalPointClouds;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TDK_ScanRegistration::mf_computeISS3DKeyPoints(const PointCloudT::Ptr &cloud_in,
                                                                                   const double &SalientRadius, const double &NonMaxRadius, const double &Gamma21, const double &Gamma32 , const double &MinNeighbors, const int &Threads)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudXYZRGBtoXYZ(cloud_in, cloud_in_xyz);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr downSampledPointCloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;

    iss_detector.setBorderRadius(0.0);

    iss_detector.setSearchMethod (tree);
    iss_detector.setSalientRadius (SalientRadius);
    iss_detector.setNonMaxRadius (NonMaxRadius);
    iss_detector.setThreshold21 (Gamma21);
    iss_detector.setThreshold32 (Gamma32);
    iss_detector.setMinNeighbors (MinNeighbors);
    iss_detector.setNumberOfThreads (Threads);

    iss_detector.setInputCloud (cloud_in_xyz);
    iss_detector.compute (*downSampledPointCloud);

    return downSampledPointCloud;
}


bool TDK_ScanRegistration::mf_processCorrespondencesSVDICP()
{
    //Downsample pointcloud and push to Downsampled vector
    mv_downSampledPointClouds.push_back(mf_computeISS3DKeyPoints(mv_originalPointClouds.back(),mv_ISS_SalientRadius, mv_ISS_NonMaxRadius,mv_ISS_Gamma21, mv_ISS_Gamma32, mv_ISS_MinNeighbors, mv_ISS_Threads));
    mv_downSampledNormals.push_back(mf_computeNormals(mv_downSampledPointClouds.back(),mv_FeatureRadiusSearch));
    //mv_downSampledFeatures.push_back(mf_computeLocalFPFH33Features(mv_downSampledPointClouds.back(), mv_downSampledNormals.back(),mv_FeatureRadiusSearch));

    
    //From VoxelICPSAC function
    if(mv_originalPointClouds.size() > 1){
        mv_downsampledCorrespondences.push_back(boost::shared_ptr<pcl::Correspondences>(mf_estimateCorrespondencesRejection(mv_downSampledPointClouds.back(),*(mv_downSampledPointClouds.end()-2),mv_SVD_MaxDistance, mv_downSampledNormals.back(),  *(mv_downSampledNormals.end()-2))));
        mf_SVDInitialAlignment();
        mf_iterativeClosestPointFinalAlignment();

        mv_downSampledNormals.back() = mf_computeNormals(mv_alignedDownSampledPointClouds.back(),mv_FeatureRadiusSearch);
        //mv_downSampledFeatures.back() = mf_computeLocalFPFH33Features(mv_alignedDownSampledPointClouds.back(), mv_downSampledNormals.back(),mv_FeatureRadiusSearch);
    }else{
        mv_alignedDownSampledPointClouds.push_back(mv_downSampledPointClouds.back());
        mv_alignedOriginalPointClouds.push_back(mv_originalPointClouds.back());
    }
    //qDebug() << "Donwsampled Point Cloud Dimension is " << (--mv_downSampledPointClouds.end())->get()->points.size()  <<endl;
    //qDebug() << "features Dimension is " << mv_downSampledFeatures.back().get()->points.size()  <<endl;

    return true;
}


pcl::CorrespondencesPtr TDK_ScanRegistration::mf_estimateCorrespondencesRejection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2, const double &max_distance, pcl::PointCloud<pcl::Normal>::Ptr &normals1, pcl::PointCloud<pcl::Normal>::Ptr &normals2)
{
    pcl::registration::CorrespondenceEstimationBackProjection<pcl::PointXYZ, pcl::PointXYZ, pcl::Normal> corr_est;

    corr_est.setInputSource(cloud1);
    corr_est.setSourceNormals(normals1);

    corr_est.setInputTarget(cloud2);
    corr_est.setTargetNormals(normals2);

    pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences());
    corr_est.determineReciprocalCorrespondences(*all_correspondences);

    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector;
    rejector.setInputSource(cloud1);
    rejector.setInputTarget(cloud2);

    //Add source and target pointcloud data to rejector?
    pcl::CorrespondencesPtr remaining_correspondences(new pcl::Correspondences());
    rejector.getRemainingCorrespondences(*all_correspondences, *remaining_correspondences);

    qDebug() << "Original corresp size: " << all_correspondences->size() << " -> " << remaining_correspondences->size() ;

    return remaining_correspondences;
}

void TDK_ScanRegistration::mf_SVDInitialAlignment()
{
    //Estimate transformation that converts downSampled -> alignedDownsampled
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> TESVD;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformation_matrix;
    TESVD.estimateRigidTransformation(*mv_downSampledPointClouds.back(),*mv_alignedDownSampledPointClouds.back(), *mv_downsampledCorrespondences.back(), transformation_matrix);

    mv_transformationMatrixAlignment.push_back(transformation_matrix);

    //Apply transformation to downsampledPC and add it to alignedDownSampledPointclouds
    PointCloudXYZ::Ptr alignedDownSampledSource(new PointCloudXYZ);
    pcl::transformPointCloud(*mv_downSampledPointClouds.back(), *alignedDownSampledSource, mv_transformationMatrixAlignment.back());
    mv_alignedDownSampledPointClouds.push_back(alignedDownSampledSource);

}



void PointCloudXYZRGBtoXYZ(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in, pcl::PointCloud<pcl::PointXYZ>::Ptr &out){
    out->empty();
    out->points.resize(in->points.size());
    for (size_t i = 0; i < out->points.size(); i++) {
        out->points[i].x = in->points[i].x;
        out->points[i].y = in->points[i].y;
        out->points[i].z = in->points[i].z;
    }
}


bool TDK_ScanRegistration::mf_processVoxelIcp()
{
    //Downsample pointcloud and push to Downsampled vector
    mv_downSampledPointClouds.push_back(mf_voxelDownSamplePointCloud(mv_originalPointClouds.back(), mv_voxelSideLength));

    if(mv_originalPointClouds.size() > 1){
        mv_alignedDownSampledPointClouds.push_back(mv_downSampledPointClouds.back());
        mv_transformationMatrixAlignment.push_back(Eigen::Matrix<float, 4, 4>::Identity());
        mf_iterativeClosestPointFinalAlignment();

    }else{
        mv_alignedDownSampledPointClouds.push_back(mv_downSampledPointClouds.back());
        mv_alignedOriginalPointClouds.push_back(mv_originalPointClouds.back());
    }
    //qDebug() << "Donwsampled Point Cloud Dimension is " << (--mv_downSampledPointClouds.end())->get()->points.size()  <<endl;
    //qDebug() << "features Dimension is " << mv_downSampledFeatures.back().get()->points.size()  <<endl;

    return true;
}
