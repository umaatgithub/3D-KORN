#include "tdk_scanregistration.h"

#include <QDebug>


using namespace std;

TDK_ScanRegistration::TDK_ScanRegistration()
{
    //Empty constructor
    mv_scannerCenterRotationSet = false;
    mv_accumulatedRotation = 0.0;


    mv_ISS_resolution = 0.003;  //0.004 ,767 points
    mv_ISS_SalientRadius = 6*mv_ISS_resolution;
    mv_ISS_NonMaxRadius = 4*mv_ISS_resolution;
    mv_ISS_Gamma21 =0.975;
    mv_ISS_Gamma32 =0.975;
    mv_ISS_MinNeighbors = 5; //5
    mv_ISS_Threads = 1;

    mv_normalRadiusSearch=0.03;

    mv_SVD_MaxDistance=0.20;

    mv_ICP_MaxCorrespondenceDistance = 0.05;
}

bool TDK_ScanRegistration::addNextPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointcloud)
{

    mv_originalPCs.push_back(inputPointcloud);

    mv_originalDenoisedPCs.push_back(mf_denoisePointCloud(mv_originalPCs.back()));

    qDebug() << "Original Points " << (--mv_originalPCs.end())->get()->points.size() <<
                " -> Denoised Points " << (--mv_originalDenoisedPCs.end())->get()->points.size();

    mf_processCorrespondencesSVDICP();

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

        pcl::transformPointCloud(*inputPointcloud, *transformedInputPointcloud, transform.matrix());

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
    return mv_downSampledPCs.back();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr TDK_ScanRegistration::mf_getMergedAlignedPC()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedAlignedOriginal(new pcl::PointCloud<pcl::PointXYZRGB>());

    vector<PointCloudT::Ptr>::iterator it;
    for (it = mv_alignedOriginalPCs.begin(); it != mv_alignedOriginalPCs.end(); ++it)
        *mergedAlignedOriginal += *(*it);

    return mf_denoisePointCloud(mergedAlignedOriginal, 1.0);

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr TDK_ScanRegistration::mf_getMergedPostRegisteredPC()
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedAlignedOriginal(new pcl::PointCloud<pcl::PointXYZRGB>());

    Eigen::Matrix4f transform;
    vector<PointCloudT::Ptr>::iterator it = mv_alignedOriginalPCs.begin();

    //Initialize PC


    int pc_num = 1;
    qDebug() << "Postprocessing " << 100*pc_num++/(mv_alignedOriginalPCs.size()-2)<< " %";

    mergedAlignedOriginal = mf_denoisePointCloud((*it)); it++;
    for ( ; it != mv_alignedOriginalPCs.end(); ++it){
        qDebug() << "Postprocessing " << 100*pc_num++/(mv_alignedOriginalPCs.size()-2)<< " %";

        *mergedAlignedOriginal += *mf_iterativeClosestPointFinalAlignment<pcl::PointXYZRGB>(
                    mf_denoisePointCloud((*it)), mergedAlignedOriginal, 0.02, transform);
    }

    return mf_denoisePointCloud(mergedAlignedOriginal, 2.5);
}

vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* TDK_ScanRegistration::mf_getAlignedPointClouds()
{
    //        mv_alignedOriginalPCs.front() = mf_denoisePointCloud(mv_alignedOriginalPCs.front(), 0.3);
    //        mv_alignedOriginalPCs.pop_back();
    return &mv_alignedOriginalPCs;
}

vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* TDK_ScanRegistration::mf_getOriginalPointClouds()
{
    return &mv_originalPCs;
}


bool TDK_ScanRegistration::mf_processCorrespondencesSVDICP()
{
    //Downsample pointcloud and push to Downsampled vector
    mv_downSampledPCs.push_back(
                mf_computeISS3DKeyPoints(
                    mv_originalDenoisedPCs.back(),
                    mv_ISS_SalientRadius, mv_ISS_NonMaxRadius,
                    mv_ISS_Gamma21, mv_ISS_Gamma32,
                    mv_ISS_MinNeighbors, mv_ISS_Threads
                    )
                );

    //Compute normals for correspondence estimation
    mv_downSampledNormals.push_back(
                mf_computeNormals(
                    mv_downSampledPCs.back(),
                    mv_normalRadiusSearch
                    )
                );


    if(mv_originalPCs.size() > 1){
        //Compute correspondences between new pointcloud and last aligned pointcloud
        mv_downsampledCorrespondences.push_back(
                    mf_estimateCorrespondences(
                        mv_downSampledPCs.back(),     *(mv_downSampledPCs.end()-2),
                        mv_downSampledNormals.back(), *(mv_downSampledNormals.end()-2),
                        mv_SVD_MaxDistance
                        )
                    );

        Eigen::Matrix4f SVDtransform;
        mv_alignedDownSampledPCs.push_back(
                    mf_SVDInitialAlignment(
                        mv_downSampledPCs.back(),        //New DSPC to be aligned
                        mv_alignedDownSampledPCs.back(), //Previous DSAPC
                        mv_downsampledCorrespondences.back(),
                        SVDtransform
                        )
                    );

        //Store rough alignment transform
        mv_transformationMatrices.push_back(SVDtransform);

        Eigen::Matrix4f ICPtransform;
        mv_alignedDownSampledPCs.back() =
                mf_iterativeClosestPointFinalAlignment<pcl::PointXYZ>(
                    *(mv_alignedDownSampledPCs.end()-1),
                    *(mv_alignedDownSampledPCs.end()-2),
                    mv_ICP_MaxCorrespondenceDistance,
                    ICPtransform
                    );

        //Merge initial transform and fine transform
        mv_transformationMatrices.back() = ICPtransform * mv_transformationMatrices.back();

        //Align original pointcloud with
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr alignedOriginal(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*mv_originalPCs.back(), *alignedOriginal, mv_transformationMatrices.back());
        mv_alignedOriginalPCs.push_back(alignedOriginal);

        //Recompute Normals once the Downsample pointcloud has been aligned
        mv_downSampledNormals.back() =
                mf_computeNormals(
                    mv_alignedDownSampledPCs.back(),
                    mv_normalRadiusSearch
                    );
    }else{
        //If its the first pointcloud, no need to align
        mv_alignedOriginalPCs.push_back(mv_originalPCs.back());
        mv_alignedDownSampledPCs.push_back(mv_downSampledPCs.back());
    }

    return true;
}



pcl::PointCloud<pcl::PointXYZ>::Ptr TDK_ScanRegistration::mf_computeISS3DKeyPoints(
        const PointCloudT::Ptr &cloud_in,
        const double &SalientRadius,
        const double &NonMaxRadius,
        const double &Gamma21,
        const double &Gamma32 ,
        const double &MinNeighbors,
        const int &Threads)
{
    //Convert pointcloud to PointXYZ (as ISS cannot operate on PointXYZRGB)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudXYZRGBtoXYZ(cloud_in, cloud_in_xyz);

    //Instantiate ISS keypoints detector
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;

    //Configure ISS  keypoints detector
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    iss_detector.setSearchMethod (tree);
    iss_detector.setSalientRadius (SalientRadius);
    iss_detector.setNonMaxRadius (NonMaxRadius);
    iss_detector.setThreshold21 (Gamma21);
    iss_detector.setThreshold32 (Gamma32);
    iss_detector.setMinNeighbors (MinNeighbors);
    iss_detector.setNumberOfThreads (Threads);

    //Compute keypoints
    iss_detector.setInputCloud (cloud_in_xyz);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downSampledPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    iss_detector.compute (*downSampledPointCloud);

    return downSampledPointCloud;
}


pcl::PointCloud<pcl::Normal>::Ptr TDK_ScanRegistration::mf_computeNormals(
        const PointCloudXYZ::Ptr &cloud_in,
        const float &searchRadius
        )
{
    //Create output pointer
    SurfaceNormalsT::Ptr normals(new SurfaceNormalsT);

    //Instantiate Normal Estimator
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;

    //Configure estimator
    norm_est.setInputCloud (cloud_in);
    norm_est.setRadiusSearch (searchRadius);

    norm_est.compute (*normals);

    return normals;
}

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
    icp.setMaximumIterations(100); //originally it was 10

    icp.setInputCloud(source);
    icp.setInputTarget(target);

    pcl::PointCloud<PointT>::Ptr alignedSource(new pcl::PointCloud<PointT>);
    icp.align(*alignedSource);

    icpTransformation = icp.getFinalTransformation();
    return alignedSource;
}


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

    //Add source and target pointcloud data to rejector?
    pcl::CorrespondencesPtr remaining_correspondences(new pcl::Correspondences());
    rejector.getRemainingCorrespondences(*all_correspondences, *remaining_correspondences);

    qDebug() << "Original corresp size: " << all_correspondences->size() << " -> " << remaining_correspondences->size() ;

    return remaining_correspondences;
}

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

    pcl::transformPointCloud(*source, *source, transformation_matrix);

    PointCloudXYZ::Ptr alignedSource(new PointCloudXYZ());
    pcl::transformPointCloud(*source, *alignedSource, transformation_matrix);

    return alignedSource;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
TDK_ScanRegistration::mf_denoisePointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
        const float std_dev
        )
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter (true);
    sorfilter.setInputCloud (cloud_in);
    sorfilter.setMeanK (8);
    sorfilter.setStddevMulThresh (std_dev);
    sorfilter.filter (*cloud_out);

    return cloud_out;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
TDK_ScanRegistration::mf_denoisePointCloud(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
        const float std_dev)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sorfilter (true);
    sorfilter.setInputCloud (cloud_in);
    sorfilter.setMeanK (8);
    sorfilter.setStddevMulThresh (std_dev);
    sorfilter.filter (*cloud_out);

    return cloud_out;
}

void
TDK_ScanRegistration::setMv_ISS_resolution(double value)
{
    mv_ISS_resolution = value;
}

void
TDK_ScanRegistration::setMv_scannerCenterRotation(const pcl::PointXYZ &value)
{
    mv_scannerCenterRotationSet = true;
    mv_scannerCenterRotation = value;
}

void
TDK_ScanRegistration::setMv_SVD_MaxDistance(double value)
{
    mv_SVD_MaxDistance = value;
}

void
TDK_ScanRegistration::setMv_ICP_MaxCorrespondenceDistance(float value)
{
    mv_ICP_MaxCorrespondenceDistance = value;
}

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

double
computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::search::KdTree<pcl::PointXYZRGB> tree;
    tree.setInputCloud (cloud);

    for (size_t i = 0; i < cloud->size (); ++i)
    {
        if (! pcl_isfinite ((*cloud)[i].x)) continue;

        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2)
        {
            res += sqrt (sqr_distances[1]);
            ++n_points;
        }
    }

    if (n_points != 0) res /= n_points;

    return res;
}
