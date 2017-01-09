#include "tdk_scanregistration.h"

#include <QDebug>


using namespace std;

TDK_ScanRegistration::TDK_ScanRegistration()
{
    //Empty constructor
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

TDK_ScanRegistration::TDK_ScanRegistration(const pcl::PointWithViewpoint scannerCenter){
    TDK_ScanRegistration();
    setScannerCenter(scannerCenter);
}

TDK_ScanRegistration::~TDK_ScanRegistration()
{
    //Destructor is empty since all of the dynamic allocation
    //is performed with boost smart pointers
}

/////////////////////////////////////////////////////
/// \brief TDK_ScanRegistration::addNextPointCloud adds and aligns one pointcloud to previous one
/// \param inputPointcloud pointcloud to be aligned with respect to previously added
/// \return
///

bool TDK_ScanRegistration::addNextPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointcloud)
{

    //Add reference of original pointcloud to array
    mv_originalPCs.push_back(inputPointcloud);

    //Remove outliers and store reference to denoised Pointcloud
    mv_originalDenoisedPCs.push_back(mf_outlierRemovalPC(mv_originalPCs.back()));

    //Call process that will roughly align the last pointcloud to all previous ones

    mf_processCorrespondencesSVDICP();

    return true;
}

/////////////////////////////////////////////////////
/// \brief TDK_ScanRegistration::addNextPointCloud adds and aligns one pointcloud to previous one compensating rotation
/// \param inputPointcloud pointcloud to be aligned with respect to previously added
/// \param degreesRotatedY degrees that current pointcloud has rotated with respect to previous
/// \return
///
bool TDK_ScanRegistration::addNextPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointcloud, const float degreesRotatedY)

{
    if(mv_scannerCenterRotationSet){
        //Transform pointcloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedInputPointcloud(new  pcl::PointCloud<pcl::PointXYZRGB>());


        //Create transform matrix that compensates for turning table orientation and distance, and rotation
        Eigen::Transform<float,3,Eigen::Affine> transform =
                Eigen::AngleAxisf(mv_scannerCenter.vp_z*(M_PI/180.0), Eigen::Vector3f::UnitZ()) *
                Eigen::AngleAxisf(mv_accumulatedRotation*(M_PI/180.0), Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(mv_scannerCenter.vp_x*(M_PI/180.0), Eigen::Vector3f::UnitX())* //20 for pamir
                Eigen::Translation3f(-mv_scannerCenter.x, 0.0, -mv_scannerCenter.z);


        pcl::transformPointCloud(*inputPointcloud, *transformedInputPointcloud, transform.matrix());

        addNextPointCloud(transformedInputPointcloud);

        mv_accumulatedRotation += degreesRotatedY;
        return true;
    }else{
        qWarning() << "Scanner Center Rotation NOT SET, To prerotate pcs you need to set it";
        return false;
    }
}


/////////////////////////////////////////////////////
/// \brief TDK_ScanRegistration::addAllPointClouds
/// \param inputPCs
/// \param degreesRotatedY
/// \return
///
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
/// \brief TDK_ScanRegistration::getLastDownSampledPointcloud
/// \return
///
pcl::PointCloud<pcl::PointXYZ>::Ptr TDK_ScanRegistration::getLastDownSampledPointcloud()
{
    return mv_downSampledPCs.back();
}

/////////////////////////////////////////////////////
/// \brief TDK_ScanRegistration::mf_getMergedAlignedPC
/// \return
///

pcl::PointCloud<pcl::PointXYZRGB>::Ptr TDK_ScanRegistration::mf_getMergedAlignedPC()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedAlignedOriginal(new pcl::PointCloud<pcl::PointXYZRGB>());

    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator it;
    for (it = mv_alignedOriginalPCs.begin(); it != mv_alignedOriginalPCs.end(); ++it)
        *mergedAlignedOriginal += *(*it);

    return mf_outlierRemovalPC(mergedAlignedOriginal, 1.0);
}

/////////////////////////////////////////////////////
/// \brief TDK_ScanRegistration::mf_getMergedPostRegisteredPC
/// \return
///

pcl::PointCloud<pcl::PointXYZRGB>::Ptr TDK_ScanRegistration::mf_getMergedPostRegisteredPC()
{
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr icp(
                new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>());
    icp->setMaxCorrespondenceDistance(mv_ICPPost_MaxCorrespondanceDistance);
    icp->setMaximumIterations (300);

    icp->setTransformationEpsilon (1e-8);

    pcl::registration::ELCH<pcl::PointXYZRGB> elch;
    elch.setReg (icp);


    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator it;
    for (it = mv_alignedOriginalPCs.begin(); it != mv_alignedOriginalPCs.end(); ++it)
        elch.addPointCloud( *it );

    elch.setLoopEnd (mv_alignedOriginalPCs.size()-1);
    elch.compute();

    //Create and setup the Incremental registration object
    pcl::registration::IncrementalRegistration<pcl::PointXYZRGB> incremental_icp;
    incremental_icp.setRegistration(icp);

    //Result pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedAlignedOriginal(new pcl::PointCloud<pcl::PointXYZRGB>());
    //Temporal pointcloud for transforming the incrementally registered
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);

    for (it = mv_alignedOriginalPCs.begin(); it != mv_alignedOriginalPCs.end(); ++it){
        incremental_icp.registerCloud( *it );

        //Align current pointcloud to previous ones
        pcl::transformPointCloud ( *(*it) , *tmp, incremental_icp.getDeltaTransform());

        *mergedAlignedOriginal += *tmp;
    }

    return mf_outlierRemovalPC(mergedAlignedOriginal, 8, 2);
}

/////////////////////////////////////////////////////
/// \brief TDK_ScanRegistration::mf_getAlignedPointClouds
/// \return
///
vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* TDK_ScanRegistration::mf_getAlignedPointClouds()
{
    return &mv_alignedOriginalPCs;
}


/////////////////////////////////////////////////////
/// \brief TDK_ScanRegistration::mf_getOriginalPointClouds
/// \return
///
vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* TDK_ScanRegistration::mf_getOriginalPointClouds()
{
    return &mv_originalPCs;
}

/////////////////////////////////////////////////////
/// \brief TDK_ScanRegistration::mf_processCorrespondencesSVDICP
/// \return
///
bool TDK_ScanRegistration::mf_processCorrespondencesSVDICP()
{

    //Downsample and add pointcloud to array
    mv_downSampledPCs.push_back(
                mf_voxelDownSamplePointCloud(
                    mv_originalDenoisedPCs.back(), mv_voxelSideLength
                    )
                );

    //Compute normals from downsampled pointcloud for correspondence estimation
    mv_downSampledNormals.push_back(
                mf_computeNormals(
                    mv_downSampledPCs.back(), mv_normalRadiusSearch
                    )
                );

    //If pointcloud array has been initialized
    if(mv_originalPCs.size() > 1){
        //Compute correspondences between new pointcloud and last aligned pointcloud
        mv_downsampledCorrespondences.push_back(
                    mf_estimateCorrespondences(
                        mv_downSampledPCs.back(),     *(mv_downSampledPCs.end()-2),
                        mv_downSampledNormals.back(), *(mv_downSampledNormals.end()-2),
                        mv_SVD_MaxDistance
                        )
                    );


        //Align downsampled pointclouds using SVD and get transform to apply on original later
        Eigen::Matrix4f SVDtransform;
        mv_alignedDownSampledPCs.push_back(
                    mf_SVDInitialAlignment(
                        mv_downSampledPCs.back(),        //New DownsampledPointclout to be aligned
                        mv_alignedDownSampledPCs.back(), //Previous DownsampledAlignedPointcloud
                        mv_downsampledCorrespondences.back(), SVDtransform
                        )
                    );

        //Store rough alignment transform for use later
        mv_transformationMatrices.push_back(SVDtransform);

        //Perform second step of initial alignment with ICP
        Eigen::Matrix4f ICPtransform;
        mv_alignedDownSampledPCs.back() =
                mf_iterativeClosestPointFinalAlignment<pcl::PointXYZ>(
                    *(mv_alignedDownSampledPCs.end()-1),
                    *(mv_alignedDownSampledPCs.end()-2),
                    mv_ICP_MaxCorrespondenceDistance, ICPtransform
                    );

        //Add both transforms together to make initial alignment transform
        mv_transformationMatrices.back() = ICPtransform * mv_transformationMatrices.back();

        //Apply compound transformation to roughly align original pointcloud to previous one
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr alignedOriginal(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*mv_originalDenoisedPCs.back(), *alignedOriginal, mv_transformationMatrices.back());
        mv_alignedOriginalPCs.push_back(alignedOriginal);

        //Recompute Normals once the Downsampled pointcloud has been aligned for use later
        mv_downSampledNormals.back() =
                mf_computeNormals(
                    mv_alignedDownSampledPCs.back(),
                    mv_normalRadiusSearch
                    );
    }else{
        //If its the first pointcloud, no need to align anything (it is already aligned to itself)
        mv_alignedOriginalPCs.push_back(mv_originalPCs.back());
        mv_alignedDownSampledPCs.push_back(mv_downSampledPCs.back());
    }

    return true;
}


/////////////////////////////////////////////////////
/// \brief TDK_ScanRegistration::mf_voxelDownSamplePointCloud
/// \param cloud_in
/// \param voxelSideLength
/// \return
///
pcl::PointCloud<pcl::PointXYZ>::Ptr TDK_ScanRegistration::mf_voxelDownSamplePointCloud(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
        const float &voxelSideLength
        )
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr downSampledPointCloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudXYZRGBtoXYZ(cloud_in, cloud_in_xyz);

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize (voxelSideLength, voxelSideLength, voxelSideLength);

    vg.setInputCloud (cloud_in_xyz);
    vg.filter (*downSampledPointCloud);

    qDebug() << "DownsampledPoints " << downSampledPointCloud->points.size();
    return downSampledPointCloud;
}


/////////////////////////////////////////////////////
/// \brief TDK_ScanRegistration::mf_computeNormals
/// \param cloud_in
/// \param searchRadius
/// \return
///
pcl::PointCloud<pcl::Normal>::Ptr TDK_ScanRegistration::mf_computeNormals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,

        const float &searchRadius
        )
{
    //Create output pointer
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    //Instantiate Normal Estimator
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;

    //Configure estimator
    norm_est.setInputCloud (cloud_in);
    norm_est.setRadiusSearch (searchRadius);

    norm_est.compute (*normals);

    return normals;
}

/////////////////////////////////////////////////////
///
///
template <typename PointT>
boost::shared_ptr<pcl::PointCloud<PointT>>
TDK_ScanRegistration::mf_iterativeClosestPointFinalAlignment(
        const boost::shared_ptr<pcl::PointCloud<PointT>> &source,
        const boost::shared_ptr<pcl::PointCloud<PointT>> &target,
        const float &maxCorrespondenceDistance,
        Eigen::Matrix4f &icpTransformation
        )
{
    pcl::IterativeClosestPoint<PointT, PointT> icp;

    icp.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
    icp.setMaximumIterations (300); //Originally 10
    icp.setTransformationEpsilon (1e-8);

    icp.setInputCloud(source);
    icp.setInputTarget(target);

    pcl::PointCloud<PointT>::Ptr alignedSource(new pcl::PointCloud<PointT>);
    icp.align(*alignedSource);

    qDebug() << "Epsilon downsampled ICP: " << icp.getFitnessScore();
    
    icpTransformation = icp.getFinalTransformation();
    return alignedSource;
}


/////////////////////////////////////////////////////
/// \brief TDK_ScanRegistration::mf_estimateCorrespondences
/// \param cloud1
/// \param cloud2
/// \param normals1
/// \param normals2
/// \param max_distance
/// \return
///

pcl::CorrespondencesPtr
TDK_ScanRegistration::mf_estimateCorrespondences(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2,
        const pcl::PointCloud<pcl::Normal>::Ptr &normals1,
        const pcl::PointCloud<pcl::Normal>::Ptr &normals2,
        const double &max_distance
        )
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
    rejector.setInputCorrespondences(all_correspondences);

    //Add source and target pointcloud data to rejector?
    pcl::CorrespondencesPtr remaining_correspondences(new pcl::Correspondences());
    rejector.getCorrespondences(*remaining_correspondences);

    qDebug() << "Original corresp size: " << all_correspondences->size() << " -> " << remaining_correspondences->size() ;

    return remaining_correspondences;
}


/////////////////////////////////////////////////////
/// \brief TDK_ScanRegistration::mf_SVDInitialAlignment
/// \param source
/// \param target
/// \param correspondences
/// \param transformation_matrix
/// \return
///

pcl::PointCloud<pcl::PointXYZ>::Ptr
TDK_ScanRegistration::mf_SVDInitialAlignment
(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
        pcl::CorrespondencesPtr correspondences,
        Eigen::Matrix4f &transformation_matrix
        )
{
    transformation_matrix = Eigen::Matrix4f ();

    //Estimate transformation that converts cloud1 -> cloud2
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> TESVD;
    TESVD.estimateRigidTransformation(*source, *target, *correspondences, transformation_matrix);

    pcl::PointCloud<pcl::PointXYZ>::Ptr alignedSource(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*source, *alignedSource, transformation_matrix);

    return alignedSource;
}


/////////////////////////////////////////////////////
/// \brief TDK_ScanRegistration::mf_outlierRemovalPC
/// \param cloud_in
/// \param meanK
/// \param std_dev
/// \return
///
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
/// \brief TDK_ScanRegistration::mf_outlierRemovalPC
/// \param cloud_in
/// \param meanK
/// \param std_dev
/// \return
///
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
/// \brief TDK_ScanRegistration::setMv_scannerCenterRotation
/// \param value
///
void
TDK_ScanRegistration::setScannerCenter(const pcl::PointWithViewpoint &value)
{
    mv_scannerCenterRotationSet = true;
    mv_scannerCenter = value;
}

pcl::PointWithViewpoint TDK_ScanRegistration::getMv_scannerCenter() const
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

void TDK_ScanRegistration::set_voxelSideLength(float value)
{
    mv_voxelSideLength = value;
}

/////////////////////////////////////////////////////
/// \brief TDK_ScanRegistration::setMv_ICPPost_MaxCorrespondanceDistance
/// \param value
///
void TDK_ScanRegistration::set_PostICP_MaxCorrespondanceDistance(float value)
{
    mv_ICPPost_MaxCorrespondanceDistance = value;
}

void TDK_ScanRegistration::set_voxelSideLength(float value)
{
    mv_voxelSideLength = value;
}

double TDK_ScanRegistration::get_SVD_MaxDistance() const
{
    return mv_SVD_MaxDistance;
}

void TDK_ScanRegistration::set_SVD_MaxDistance(double value)
{
    mv_SVD_MaxDistance = value;
}

float TDK_ScanRegistration::get_ICP_MaxCorrespondenceDistance() const
{
    return mv_ICP_MaxCorrespondenceDistance;
}
void TDK_ScanRegistration::set_ICP_MaxCorrespondenceDistance(float value)
{
    mv_ICP_MaxCorrespondenceDistance = value;
}

float TDK_ScanRegistration::get_ICPPost_MaxCorrespondanceDistance() const
{
    return mv_ICPPost_MaxCorrespondanceDistance;
}

void TDK_ScanRegistration::set_ICPPost_MaxCorrespondanceDistance(float value)
{
    mv_ICPPost_MaxCorrespondanceDistance = value;
}

void
TDK_ScanRegistration::setMv_scannerCenterRotation(const pcl::PointXYZ &value)
{
    mv_scannerCenterRotationSet = true;
    mv_scannerCenterRotation = value;
}


/////////////////////////////////////////////////////
/// \brief TDK_ScanRegistration::setMv_SVD_MaxDistance
/// \param value
///
void
TDK_ScanRegistration::set_SVD_MaxDistance(double value)

{
    mv_SVD_MaxDistance = value;
}


/////////////////////////////////////////////////////
/// \brief TDK_ScanRegistration::setMv_ICP_MaxCorrespondenceDistance
/// \param value
///
void
TDK_ScanRegistration::set_ICP_MaxCorrespondenceDistance(float value)
{
    mv_ICP_MaxCorrespondenceDistance = value;
}

/////////////////////////////////////////////////////
/// \brief PointCloudXYZRGBtoXYZ
/// \param in
/// \param out
///
void
PointCloudXYZRGBtoXYZ(
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
