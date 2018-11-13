
#include "tdk_meshing.h"
using namespace pcl;

TDK_Meshing::TDK_Meshing()
{

}

TDK_Meshing::~TDK_Meshing(){

}

//With NormalEstimationFunction we normalize the output received from the FilterPassThrough
//mf_NormalEstimation receives as input the filtered PointCloud and gives as output a PointNormal
void TDK_Meshing::mf_NormalEstimation(PointCloud<PointXYZ>::Ptr &mv_PointCloudInput,
                                              PointCloud<pcl::PointNormal>::Ptr &mv_PointNormalOutput){

    pcl::search::KdTree<pcl::PointXYZ>::Ptr mv_Tree;
    mv_Tree.reset(new pcl::search:: KdTree<pcl::PointXYZ>(false) ) ;
    mv_Tree->setInputCloud(mv_PointCloudInput);
    cout << "begin normal estimation" << endl;
    NormalEstimationOMP<PointXYZ, Normal> mv_Normal;

    mv_Normal.setNumberOfThreads(8);
    mv_Normal.setSearchMethod(mv_Tree);
    mv_Normal.setInputCloud(mv_PointCloudInput);
    mv_Normal.setRadiusSearch(0.3);
    Eigen::Vector4f centroid;
    compute3DCentroid(*mv_PointCloudInput, centroid);
    mv_Normal.setViewPoint(centroid[0], centroid[1], centroid[2]);
    PointCloud<Normal>::Ptr mv_CloudNormals (new PointCloud<Normal>());
    mv_Normal.compute(*mv_CloudNormals);

    cout << "normal estimation complete" << endl;

    cout << "reverse normals' direction" << endl;
    for(size_t i = 0; i < mv_CloudNormals->size(); ++i){
        mv_CloudNormals->points[i].normal_x *= -1;
        mv_CloudNormals->points[i].normal_y *= -1;
        mv_CloudNormals->points[i].normal_z *= -1;
    }

    cout << "combine points and normals" << endl;
    pcl::concatenateFields(*mv_PointCloudInput, *mv_CloudNormals, *mv_PointNormalOutput);
    qDebug()<<"normal estimation finished";
}


void TDK_Meshing::mf_ConvertFromXYZRGBtoXYZ(const PointCloud<pcl::PointXYZRGB>::Ptr &mv_PointCloudInput,
                                            PointCloud<pcl::PointXYZ>::Ptr &mv_PointCloudOutput){
    pcl::copyPointCloud(*mv_PointCloudInput, *mv_PointCloudOutput);
}


void TDK_Meshing::mf_Poisson(const PointCloud<PointXYZ>::Ptr &mv_PointCloudInput,
                                   pcl::PolygonMesh::Ptr &mv_MeshesOutput1){
    
    //Perform statistical outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    TDK_Filters::mf_FilterStatisticalOutlierRemoval(mv_PointCloudInput, mv_cloud_filtered, 5);
    
    //Apply the MLS smoothing filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_cloud_smoothed (new  pcl::PointCloud<pcl::PointXYZ>);
    TDK_Filters::mf_FilterMLSSmoothing(mv_cloud_filtered, mv_cloud_smoothed, 0.07);

    //Obtain a normal point estimation
    pcl::PointCloud<pcl::PointNormal>::Ptr mv_PointNormal1(new pcl::PointCloud<pcl::PointNormal>());
    TDK_Meshing::mf_NormalEstimation(mv_cloud_smoothed, mv_PointNormal1);

    //Poisson Function
    pcl::Poisson<PointNormal> mv_Poisson;
    mv_Poisson.setInputCloud(mv_PointNormal1);
    mv_Poisson.setDepth(9);
    mv_Poisson.setScale(1.2);
    mv_Poisson.setSolverDivide(8);
    mv_Poisson.setIsoDivide(8);
    mv_Poisson.setSamplesPerNode(1);
    mv_Poisson.setConfidence(false);
    mv_Poisson.setManifold(true);
    mv_Poisson.setOutputPolygons(false);

    boost::shared_ptr<pcl::PolygonMesh> mesh (new pcl::PolygonMesh);
    mv_Poisson.reconstruct(*mesh);

    //Laplacian Smoothing of mesh
    TDK_Filters::mf_FilterLaplacianSmoothing(mesh, mv_MeshesOutput1);
    qDebug()<<"inside poisson";
}

void TDK_Meshing::mf_Poisson(const PointCloud<PointXYZRGB>::Ptr &mv_PointCloudInputRGB,
                                   pcl::PolygonMesh::Ptr &mv_MeshesOutput1){

    //Convert from RGBXYZ to XYZ
    PointCloud<PointXYZ>::Ptr mv_PointCloudInput  (new PointCloud<PointXYZ>) ;
    TDK_Meshing::mf_ConvertFromXYZRGBtoXYZ(mv_PointCloudInputRGB, mv_PointCloudInput) ;
    
    //Perform statistical outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    TDK_Filters::mf_FilterStatisticalOutlierRemoval(mv_PointCloudInput, mv_cloud_filtered, 5);
    
    //Apply the MLS smoothing filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_cloud_smoothed (new  pcl::PointCloud<pcl::PointXYZ>);
    TDK_Filters::mf_FilterMLSSmoothing(mv_cloud_filtered, mv_cloud_smoothed, 0.07);

    //Obtain a normal point estimation
    pcl::PointCloud<pcl::PointNormal>::Ptr mv_PointNormal1(new pcl::PointCloud<pcl::PointNormal>());
    TDK_Meshing::mf_NormalEstimation(mv_cloud_smoothed, mv_PointNormal1);

    //Poisson Function
    pcl::Poisson<PointNormal> mv_Poisson;
    mv_Poisson.setInputCloud(mv_PointNormal1);
    mv_Poisson.setDepth(9);
    mv_Poisson.setScale(1.2);
    mv_Poisson.setSolverDivide(8);
    mv_Poisson.setIsoDivide(8);
    mv_Poisson.setSamplesPerNode(1);
    mv_Poisson.setConfidence(false);
    mv_Poisson.setManifold(true);
    mv_Poisson.setOutputPolygons(false);

    boost::shared_ptr<pcl::PolygonMesh> mesh (new pcl::PolygonMesh);
    mv_Poisson.reconstruct(*mesh);

    //Laplacian Smoothing of mesh
    TDK_Filters::mf_FilterLaplacianSmoothing(mesh, mv_MeshesOutput1);
    qDebug()<<"inside poisson";
}

void TDK_Meshing::mf_Greedy_Projection_Triangulation(const PointCloud<PointXYZ>::Ptr &mv_PointCloudInput,
                                         pcl::PolygonMesh::Ptr &mv_MeshesOutput1){
    
    //Perform statistical outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    TDK_Filters::mf_FilterStatisticalOutlierRemoval(mv_PointCloudInput, mv_cloud_filtered, 5);
    
    //Apply the MLS smoothing filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_cloud_smoothed (new  pcl::PointCloud<pcl::PointXYZ>);
    TDK_Filters::mf_FilterMLSSmoothing(mv_cloud_filtered, mv_cloud_smoothed, 0.07);

    //Obtain a normal point estimation
    pcl::PointCloud<pcl::PointNormal>::Ptr mv_PointNormal1(new pcl::PointCloud<pcl::PointNormal>());
    TDK_Meshing::mf_NormalEstimation(mv_cloud_smoothed, mv_PointNormal1);

    //Perform Greedy Projection Triangulation
    // Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (mv_PointNormal1);
    boost::shared_ptr<pcl::PolygonMesh> mesh (new pcl::PolygonMesh);
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

    // Set typical values for the parameters
    gp3.setSearchRadius (0.025);
    gp3.setMu (3);//3
    gp3.setMaximumNearestNeighbors (1200);
    gp3.setMaximumSurfaceAngle(M_PI);
    gp3.setMinimumAngle(M_PI/36);
    gp3.setMaximumAngle(2*M_PI/3 + M_PI/6);
    gp3.setNormalConsistency(true);

    // Get result
    gp3.setInputCloud (mv_PointNormal1);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (*mesh);


    //Laplacian Smoothing of mesh
    TDK_Filters::mf_FilterLaplacianSmoothing(mesh,mv_MeshesOutput1);
    qDebug()<<"inside greedy triangulation";
}

void TDK_Meshing::mf_Greedy_Projection_Triangulation(const PointCloud<PointXYZRGB>::Ptr &mv_PointCloudInputRGB,
                                         pcl::PolygonMesh::Ptr &mv_MeshesOutput1){

    //Convert from RGBXYZ to XYZ
    PointCloud<PointXYZ>::Ptr mv_PointCloudInput  (new PointCloud<PointXYZ>) ;
    TDK_Meshing::mf_ConvertFromXYZRGBtoXYZ(mv_PointCloudInputRGB, mv_PointCloudInput);

    //Perform statistical outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    TDK_Filters::mf_FilterStatisticalOutlierRemoval(mv_PointCloudInput, mv_cloud_filtered, 5);
    
    //Apply the MLS smoothing filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_cloud_smoothed (new  pcl::PointCloud<pcl::PointXYZ>);
    TDK_Filters::mf_FilterMLSSmoothing(mv_cloud_filtered, mv_cloud_smoothed, 0.07);

    //Obtain a normal point estimation
    pcl::PointCloud<pcl::PointNormal>::Ptr mv_PointNormal1(new pcl::PointCloud<pcl::PointNormal>());
    TDK_Meshing::mf_NormalEstimation(mv_cloud_smoothed, mv_PointNormal1);

    //Perform Greedy Projection Triangulation
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    // Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (mv_PointNormal1);
    boost::shared_ptr<pcl::PolygonMesh> mesh (new pcl::PolygonMesh);

    // Set typical values for the parameters
    gp3.setSearchRadius (0.025);
    gp3.setMu (3);//3
    gp3.setMaximumNearestNeighbors (1200);
    gp3.setMaximumSurfaceAngle(M_PI);
    gp3.setMinimumAngle(M_PI/36);
    gp3.setMaximumAngle(2*M_PI/3 + M_PI/6);
    gp3.setNormalConsistency(true);

    // Get result
    gp3.setInputCloud (mv_PointNormal1);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (*mesh);

    //Laplacian Smoothing of mesh
    TDK_Filters::mf_FilterLaplacianSmoothing(mesh,mv_MeshesOutput1);
    qDebug()<<"inside greedy triangulation";
}

void TDK_Meshing::mf_Grid_Projection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &mv_PointCloudInput,
                                     pcl::PolygonMesh::Ptr &mv_MeshesOutput1){
    
    //Perform statistical outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    TDK_Filters::mf_FilterStatisticalOutlierRemoval(mv_PointCloudInput, mv_cloud_filtered, 5);
    
    //Apply the MLS smoothing filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_cloud_smoothed (new  pcl::PointCloud<pcl::PointXYZ>);
    TDK_Filters::mf_FilterMLSSmoothing(mv_cloud_filtered, mv_cloud_smoothed, 0.07);

    //Obtain a normal point estimation
    pcl::PointCloud<pcl::PointNormal>::Ptr mv_PointNormal1(new pcl::PointCloud<pcl::PointNormal>());
    TDK_Meshing::mf_NormalEstimation(mv_cloud_smoothed, mv_PointNormal1);

    //Perform grid projection
    pcl::GridProjection<pcl::PointNormal> gp;
    boost::shared_ptr<pcl::PolygonMesh> mesh (new pcl::PolygonMesh);

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (mv_PointNormal1);

    gp.setInputCloud(mv_PointNormal1);
    gp.setSearchMethod(tree2);
    gp.setResolution(0.005);
    gp.setPaddingSize(3);
    gp.reconstruct(*mesh);

    //Laplacian Smoothing of mesh
    TDK_Filters::mf_FilterLaplacianSmoothing(mesh,mv_MeshesOutput1);
    qDebug()<<"inside grid projection";

}

void TDK_Meshing::mf_Grid_Projection(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &mv_PointCloudInputRGB, pcl::PolygonMesh::Ptr &mv_MeshesOutput1){
    
    //Convert from RGBXYZ to XYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_PointCloudInput  (new pcl::PointCloud<pcl::PointXYZ>) ;
    TDK_Meshing::mf_ConvertFromXYZRGBtoXYZ(mv_PointCloudInputRGB, mv_PointCloudInput) ;
    
    //Perform statistical outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    TDK_Filters::mf_FilterStatisticalOutlierRemoval(mv_PointCloudInput, mv_cloud_filtered, 5);
    
    //Apply the MLS smoothing filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_cloud_smoothed (new  pcl::PointCloud<pcl::PointXYZ>);
    TDK_Filters::mf_FilterMLSSmoothing(mv_cloud_filtered, mv_cloud_smoothed, 0.07);


    //Obtain a normal point estimation
    pcl::PointCloud<pcl::PointNormal>::Ptr mv_PointNormal1(new pcl::PointCloud<pcl::PointNormal>());
    TDK_Meshing::mf_NormalEstimation(mv_cloud_smoothed, mv_PointNormal1);

    //Perform grid projection
    pcl::GridProjection<pcl::PointNormal> gp;
    boost::shared_ptr<pcl::PolygonMesh> mesh (new pcl::PolygonMesh);

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (mv_PointNormal1);

    gp.setInputCloud(mv_PointNormal1);
    gp.setSearchMethod(tree2);
    gp.setResolution(0.005);
    gp.setPaddingSize(3);
    gp.reconstruct(*mesh);

    //Laplacian Smoothing of mesh
    TDK_Filters::mf_FilterLaplacianSmoothing(mesh,mv_MeshesOutput1);
    qDebug()<<"inside grid projection";

}

void TDK_Meshing::mf_Marching_Cubes(const pcl::PointCloud<pcl::PointXYZ>::Ptr &mv_PointCloudInput,
                                     pcl::PolygonMesh::Ptr &mv_MeshesOutput1){
    qDebug()<<"inside marching cubes";
    
    //Perform statistical outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    TDK_Filters::mf_FilterStatisticalOutlierRemoval(mv_PointCloudInput, mv_cloud_filtered, 5);
    
    //Apply the MLS smoothing filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_cloud_smoothed (new  pcl::PointCloud<pcl::PointXYZ>);
    TDK_Filters::mf_FilterMLSSmoothing(mv_cloud_filtered, mv_cloud_smoothed, 0.07);
    
    //Obtain a normal point estimation
    pcl::PointCloud<pcl::PointNormal>::Ptr mv_PointNormal1(new pcl::PointCloud<pcl::PointNormal>());
    TDK_Meshing::mf_NormalEstimation(mv_cloud_smoothed, mv_PointNormal1);
    
    //Perform marching cube reconstruction
    pcl::MarchingCubesHoppe<pcl::PointNormal> marching;
    boost::shared_ptr<pcl::PolygonMesh> mesh (new pcl::PolygonMesh);
    
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (mv_PointNormal1);
    
    marching.setInputCloud(mv_PointNormal1);
    marching.setIsoLevel(0);
    marching.setGridResolution(75,75,75);
    marching.setPercentageExtendGrid (0.3f);
    marching.setSearchMethod(tree2);
    marching.reconstruct(*mesh);
    
    //Laplacian Smoothing of mesh
    TDK_Filters::mf_FilterLaplacianSmoothing(mesh,mv_MeshesOutput1);
    qDebug()<<"finished marching cubes";
    
}

void TDK_Meshing::mf_Marching_Cubes(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &mv_PointCloudInputRGB, pcl::PolygonMesh::Ptr &mv_MeshesOutput1){
    
    //Convert from RGBXYZ to XYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_PointCloudInput  (new pcl::PointCloud<pcl::PointXYZ>) ;
    TDK_Meshing::mf_ConvertFromXYZRGBtoXYZ(mv_PointCloudInputRGB, mv_PointCloudInput) ;
    
    //Perform statistical outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    TDK_Filters::mf_FilterStatisticalOutlierRemoval(mv_PointCloudInput, mv_cloud_filtered, 5);
    
    //Apply the MLS smoothing filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_cloud_smoothed (new  pcl::PointCloud<pcl::PointXYZ>);
    TDK_Filters::mf_FilterMLSSmoothing(mv_cloud_filtered, mv_cloud_smoothed, 0.07);
    
    //Obtain a normal point estimation
    pcl::PointCloud<pcl::PointNormal>::Ptr mv_PointNormal1(new pcl::PointCloud<pcl::PointNormal>());
    TDK_Meshing::mf_NormalEstimation(mv_cloud_smoothed, mv_PointNormal1);
    
    //Perform marching cube reconstruction
    pcl::MarchingCubesHoppe<pcl::PointNormal> marching;
    boost::shared_ptr<pcl::PolygonMesh> mesh (new pcl::PolygonMesh);
    
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (mv_PointNormal1);
    
    marching.setInputCloud(mv_PointNormal1);
    marching.setIsoLevel(0);
    marching.setGridResolution(75,75,75);
    marching.setPercentageExtendGrid (0.3f);
    marching.setSearchMethod(tree2);
    marching.reconstruct(*mesh);
    
    //Laplacian Smoothing of mesh
    TDK_Filters::mf_FilterLaplacianSmoothing(mesh,mv_MeshesOutput1);
    qDebug()<<"finished marching cubes";
    
}

