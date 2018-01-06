#include "tdk_filters.h"

using namespace pcl;

TDK_Filters::TDK_Filters()
{

}


TDK_Filters::~TDK_Filters()
{

}

void TDK_Filters::FilterPCPassthrough(const double &ci, const double &minx, const double &maxx, const double &miny, const double &maxy, const double &minz, const double &maxz, uint &xi, uint &yi, uint &zi, const pcl::PointCloud<PointXYZ>::Ptr &cloud, pcl::PointCloud<PointXYZ>::Ptr &cloud_filtered){

    double xremapplus,xremapminus,yremapplus,yremapminus,zremapplus,zremapminus;

    xremapplus  =  maxx - xi*(maxx-minx)/255;
    xremapminus =  xi*(maxx-minx)/255+minx;

    yremapplus  =  maxy - yi*(maxy-miny)/255;
    yremapminus =  yi*(maxy-miny)/255+miny;

    zremapplus  =  maxz - zi*(maxz-minz)/255;
    zremapminus =  zi*(maxz-minz)/255+minz;
    PassThrough<PointXYZ> pass (true);

    if (ci == 1){
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (zremapplus, maxz);
        pass.setInputCloud (cloud);
        pass.filter (*cloud_filtered);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (yremapplus, maxy);
        pass.setInputCloud (cloud_filtered);
        pass.filter (*cloud_filtered);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (xremapplus, maxx); //maxx
        pass.setInputCloud (cloud_filtered);
        pass.filter (*cloud_filtered);
    }
    else{
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (minz,zremapminus);
        pass.setInputCloud (cloud_filtered);
        pass.filter (*cloud_filtered);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (miny,yremapminus);
        pass.setInputCloud (cloud_filtered);
        pass.filter (*cloud_filtered);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (minx,xremapminus);
        pass.setInputCloud (cloud_filtered);
        pass.filter (*cloud_filtered);
    }
}

//PassThrough filter attempt 2
//Input: PointCloud, PointCloud(filtered), x,y,z minimum and maximum
//Output: void
void TDK_Filters::mf_FilterPassthroughBri(const  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered, float x1, float x2, float y1, float y2, float z1, float z2){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredz (new  pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredy (new  pcl::PointCloud<pcl::PointXYZ>);

    // Create the passthrough filtering object
    pcl::PassThrough<PointXYZ> pass;

    qDebug()<<"inside pass through";

    pass.setInputCloud (cloud);  // passing the input cloud pointer
    pass.setFilterFieldName ("z"); // defining the dimension to be filtered
    pass.setFilterLimits (z1,z2);  // setting the limits
    pass.filter (*cloud_filteredz); // filter and put the output in a point cloud variable

    qDebug()<<"z filtered";

    pass.setInputCloud (cloud_filteredz); // passing the input cloud pointer
    pass.setFilterFieldName ("y"); // defining the dimension to be filtered
    pass.setFilterLimits (y1,y2);  // setting the limits
    pass.filter (*cloud_filteredy); // filter and put the output in a point cloud variable

    pass.setInputCloud (cloud_filteredy); // passing the input cloud pointer
    pass.setFilterFieldName ("x"); // defining the dimension to be filtered
    pass.setFilterLimits (x1, x2); // setting the limits
    pass.filter (*cloud_filtered); // filter and put the output in a point cloud variable

    qDebug()<<"x and y filtered";
}

//Statistical outlier removal:
//Input: PointCloud, PointCloud(filtered), standard deviation threshold
//Output: void
void TDK_Filters::mf_FilterStatisticalOutlierRemoval(const pcl::PointCloud<PointXYZ>::Ptr &cloud, const pcl::PointCloud<PointXYZ>::Ptr &cloud_filtered, float threshold){

    pcl::StatisticalOutlierRemoval<PointXYZ> sor; // Create the filtering object
    qDebug()<<"inside outlier";
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (threshold);
    sor.filter (*cloud_filtered);
}

//Statistical outlier removal:
//Input: RGBPointCloud, RGBPointCloud(filtered), standard deviation threshold
//Output: void
void TDK_Filters::mf_FilterStatisticalOutlierRemoval(const pcl::PointCloud<PointXYZRGB>::Ptr &cloud,
                                                     const pcl::PointCloud<PointXYZRGB>::Ptr &cloud_filtered, float threshold){

    pcl::StatisticalOutlierRemoval<PointXYZRGB> sor; // Create the filtering object
    qDebug()<<"inside outlier";
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (threshold);
    sor.filter (*cloud_filtered);
}



//Voxel grid downsample:
//Input: PointCloud, PointCloud(filtered), leafsize (size of cubic voxel)
//Output: void
void TDK_Filters::mf_FilterVoxelGridDownsample(const pcl::PointCloud<PointXYZ>::Ptr &cloud, pcl::PointCloud<PointXYZ>::Ptr &cloud_filtered, const float &leafsize){

      pcl::VoxelGrid<PointXYZ> sor; // Create the filtering object
      qDebug()<<"inside voxelgrid";
      sor.setInputCloud (cloud);
      sor.setLeafSize (leafsize,leafsize, leafsize); //Cubic voxel
      sor.filter (*cloud_filtered);
}

//Voxel grid downsample:
//Input: PointCloud, PointCloud(filtered), leafsize (size of cubic voxel)
//Output: void
void TDK_Filters::mf_FilterVoxelGridDownsample(const pcl::PointCloud<PointXYZRGB>::Ptr &cloud, pcl::PointCloud<PointXYZRGB>::Ptr &cloud_filtered, const float &leafsize){
    
    pcl::VoxelGrid<PointXYZRGB> sor; // Create the filtering object
    qDebug()<<"inside voxelgrid";
    sor.setInputCloud (cloud);
    sor.setLeafSize (leafsize,leafsize, leafsize); //Cubic voxel
    sor.filter (*cloud_filtered);
}

//MLS Filter Smoothing:
//Input: PointCloud, PointCloud(smoothed), searchradius (sphere radius used for k-space nearest neighbors)
//Output: void
void TDK_Filters::mf_FilterMLSSmoothing(const pcl::PointCloud<PointXYZ>::Ptr &cloud, pcl::PointCloud<PointXYZ>::Ptr &cloud_smoothed, float searchradius){

    // Start MLS
    // Create a KD-Tree
    pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ>);
    pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::MovingLeastSquares<PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);
    mls.setInputCloud(cloud);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree); //Kdtree search method
    mls.setSearchRadius(searchradius); //Set sphere radius used for k-space nearest neighbors
    mls.process(mls_points);
    pcl::copyPointCloud(mls_points, *cloud_smoothed); //convert from XYZNormals to XYZ
    qDebug()<<"finished MLS Smoothing";
}

//Laplacian Filter Smoothing:
//Input: Mesh, Mesh(smoothed)
//Output: void
void TDK_Filters::mf_FilterLaplacianSmoothing(const boost::shared_ptr<pcl::PolygonMesh> &triangles, pcl::PolygonMesh::Ptr &mv_MeshesOutput1){

    pcl::MeshSmoothingLaplacianVTK laplacian;
    laplacian.setInputMesh(triangles);
    laplacian.setNumIter(20000);
    laplacian.setConvergence(0.0001);
    laplacian.setRelaxationFactor(0.0001);
    laplacian.setFeatureEdgeSmoothing(true);
    laplacian.setFeatureAngle(M_PI/5);
    laplacian.setBoundarySmoothing(true);
    laplacian.process(*mv_MeshesOutput1);
    qDebug()<<"Laplacian smoothing Finished";
}
