#include "registration.h"


PointCloud::Ptr Register (PointCloud::Ptr src, PointCloud::Ptr tgt){


        float MaxDistance=0.015;
        float RansacVar = 0.01;
        float Iterations = 100;

        PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
        PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

        //PointCloudWithNormals::Ptr normals_icp (new PointCloudWithNormals);

        PointCloud::Ptr cloud_norm (new PointCloud);

        pcl::NormalEstimation<PointT, PointNormalT> norm_est;
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
        norm_est.setSearchMethod (tree);
        norm_est.setKSearch (12);

        norm_est.setInputCloud (src);
        norm_est.compute (*points_with_normals_src);
        pcl::copyPointCloud (*src, *points_with_normals_src);

        norm_est.setInputCloud (tgt);
        norm_est.compute (*points_with_normals_tgt);
        pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

        pcl::IterativeClosestPointWithNormals<PointNormalT, PointNormalT> reg;
        reg.setTransformationEpsilon (1e-8);

        // Set the maximum distance between two correspondences (src<->tgt) to 10cm
        // Note: adjust this based on the size of your datasets

        reg.setMaxCorrespondenceDistance (MaxDistance);
        reg.setRANSACOutlierRejectionThreshold (RansacVar); // 0.05
        reg.setMaximumIterations (Iterations);


        reg.setInputSource (points_with_normals_src);
        reg.setInputTarget (points_with_normals_tgt);
        //reg.align (*normals_icp);

        std::cout << "Normals converged with score: " << reg.getFitnessScore() << std::endl;

        Eigen::Matrix4f transform_normals = reg.getFinalTransformation ();
        pcl::transformPointCloud (*src, *cloud_norm, transform_normals);


        return cloud_norm;

  }






void Save_PointC(PointCloud::Ptr){





}
