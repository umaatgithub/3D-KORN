#include "TDK_PointOperations.h"


using namespace pcl;

TDK_PointOperations::TDK_PointOperations()
{


}

//My try ---> decide to delete it or not !
void TDK_PointOperations::FilterPCPassthrough(const double &minx, const double &maxx, const double &miny, const double &maxy, const double &minz, const double &maxz, const double &kx, const double &ky, const double &kz, uint &xi, uint &yi, uint &zi, const PointCloud<PointXYZ>::Ptr &cloud, PointCloud<PointXYZ>::Ptr &clouda){
    PassThrough<PointXYZ> pass (true);
    //EXPECT_EQ (pass.getRemovedIndices()->size(), 0);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (maxz-(zi*(maxz-minz)/255), maxz);
    pass.setInputCloud (cloud);
    pass.filter (*clouda);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (maxy-(yi*(maxy-miny)/255), maxy);
    pass.setInputCloud (clouda);
    pass.filter (*clouda);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (maxx-(xi*(maxx-minx)/255), maxx); //maxx
    pass.setInputCloud (clouda);
    pass.filter (*clouda);
        //i = maxx-(xi*(maxx-minx)/255);

}

//PassThrough Filter for Optimization
void TDK_PointOperations::mf_FilterPassthrough(const PointCloud<PointXYZ>::Ptr &PointCloudInput, PointCloud<PointXYZ>::Ptr &PointCloudOutput){
    cout << "begin passthrough filter" << endl ;
    PassThrough<PointXYZ> filter;
    filter.setInputCloud(PointCloudInput);
    filter.filter(*PointCloudOutput); //Filtered PointCloud set as OutPut
    cout << "passthrough filter complete" << endl;
}

//With NormalEstimationFunction we normalize the output received from the FilterPassThrough
//mf_NormalEstimation receives as input the filtered PointCloud and gives as output a PointNormal
void TDK_PointOperations::mf_NormalEstimation(PointCloud<PointXYZ>::Ptr &PointCloudInput, PointCloud<pcl::PointNormal>::Ptr &PointNormalOutput){

    cout << "begin normal estimation" << endl;
    NormalEstimationOMP<PointXYZ, Normal> ne;
    ne.setNumberOfThreads(8);
    ne.setInputCloud(PointCloudInput);
    ne.setRadiusSearch(0.05);
    Eigen::Vector4f centroid;
    compute3DCentroid(*PointCloudInput, centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
    PointCloud<Normal>::Ptr CloudNormals (new PointCloud<Normal>());
    ne.compute(*CloudNormals);
    cout << "normal estimation complete" << endl;

    cout << "reverse normals' direction" << endl;
    for(size_t i = 0; i < CloudNormals->size(); ++i){
        CloudNormals->points[i].normal_x *= -1;
        CloudNormals->points[i].normal_y *= -1;
        CloudNormals->points[i].normal_z *= -1;
    }

    cout << "combine points and normals" << endl;
    concatenateFields(*PointCloudInput, *CloudNormals, *PointNormalOutput);
}


//mf_PoissonMeshes takes as input a PointCloud, applies all the function defined above and gives as output a mesh
void TDK_PointOperations::mf_PoissonMeshes(const PointCloud<PointXYZ>::Ptr &PointCloudInput , PolygonMesh::Ptr &MeshesOutput){

    //PassThrough filter function
    PointCloud<PointXYZ>::Ptr PointCloudForFiltering  ((new PointCloud<PointXYZ>)) ;
    TDK_PointOperations::mf_FilterPassthrough(PointCloudInput,PointCloudForFiltering);

    //Normal Estimation function
    PointCloud<PointNormal>::Ptr PointNormal_V(new PointCloud<PointNormal>());
    TDK_PointOperations::mf_NormalEstimation(PointCloudForFiltering, PointNormal_V);

    //Poisson Function
    Poisson<PointNormal> Poisson;
    Poisson.setInputCloud(PointNormal_V);
    Poisson.setDepth(10);
    Poisson.setSolverDivide (8);
    Poisson.setIsoDivide (8);
    Poisson.setConfidence(false);
    Poisson.setManifold(true);
    Poisson.setOutputPolygons(false);
    //PolygonMesh;
    Poisson.reconstruct(*MeshesOutput);
}

   void TDK_PointOperations::mf_PoissonMeshes(const PointCloud<PointXYZRGB>::Ptr &PointCloudXYZRGB, PolygonMesh::Ptr &MeshesOutput){

    PointCloud<PointXYZ>::Ptr PointCloudInput  ((new PointCloud<PointXYZ>)) ;
    TDK_PointOperations::mf_ConvertFromXYZRGBtoXYZ(PointCloudXYZRGB, PointCloudInput) ;
    TDK_PointOperations::mf_PoissonMeshes(PointCloudInput, MeshesOutput);



}



void TDK_PointOperations::mf_ConvertFromXYZRGBtoXYZ(const PointCloud<pcl::PointXYZRGB>::Ptr &PointCloudInput, PointCloud<pcl::PointXYZ>::Ptr &PointCloudOutput){
    copyPointCloud(*PointCloudInput, *PointCloudOutput);
}
