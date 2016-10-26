// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
//#include <pcl/registration/sample_consensus_prerejective.h>
#include <QDebug>

typedef pcl::PointXYZRGBA PointType;

int main( int argc, char* argv[] )
{
    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer( "Point Cloud Viewer" ) );
    viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0, 0.0, 0.0 );

    pcl::PointCloud<PointType>::Ptr cloud1((new pcl::PointCloud<PointType>));
    pcl::PointCloud<PointType>::Ptr cloud2((new pcl::PointCloud<PointType>));

    //std::string filename1 = "../eze_2.ply";
    //std::string filename2 = "../eze_3.ply";

    pcl::io::loadPLYFile("../eze_2.ply", *cloud1);
    pcl::io::loadPLYFile("../eze_3.ply", *cloud2);

    qDebug() << "Read PlY";

    pcl::SampleConsensusInitialAlignment<PointT, PointT, DescriptorT> sac;

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputCloud(cloud1);
    icp.setInputTarget(cloud2);

    qDebug() << "Start ICP";
    pcl::PointCloud<PointType> final;
    icp.align(final);

    //final is cloud1 transformed by ICP

    int combinedSize = final.points.size() + cloud2->points.size();
    pcl::PointCloud<PointType> final2;
    final2.resize(combinedSize);

    for (int i = 0; i < final.points.size(); ++i) {
        final2.points[i] = final.points[i];
    }
    for (int i = final.points.size(); i < combinedSize; ++i) {
        final2.points[i] = (*cloud2.get()).points[i-final.points.size()];
    }
    qDebug() << final.points.size();

    qDebug() << "Finished ICP";


    // Point Cloud
    if(!icp.hasConverged()) return 0;

    qDebug() << "ICP converged!";
    pcl::io::savePLYFileBinary ("icp.ply" , final);
//    qDebug() << "Showing";

//    viewer->spinOnce();

//    viewer->addPointCloud( final, "final" );
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "final");

//    viewer->spinOnce();

    return 0;
}
