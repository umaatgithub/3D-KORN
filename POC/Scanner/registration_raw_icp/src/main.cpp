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
    // Create a boost pointer to a PCLVisualizer window
    // (boost is for something about concurrency and sharing pointers through the code)
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
            new pcl::visualization::PCLVisualizer( "Point Cloud Viewer") );
    viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0, 0.0, 0.0 );

    //Create two cloud pointers and initialize them (if we dont initialize they will point to NULL)
    pcl::PointCloud<PointType>::Ptr cloud1(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud2(new pcl::PointCloud<PointType>);

    //Load the two stored .ply files into cloud1 and cloud 2
    pcl::io::loadPLYFile("../../../models/eze_2.ply", *cloud1);
    pcl::io::loadPLYFile("../../../models/eze_3.ply", *cloud2);

    //Inform user that reading has been succesfull :D
    qDebug() << "Read PlY";

    //Create the ICP "tool" object
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    //Set cloud1 as transformable cloud for icp algorithm
    icp.setInputCloud(cloud1);
    //Set cloud2 as target cloud for icp algorithm
    icp.setInputTarget(cloud2);

    qDebug() << "Start ICP";
    //Create pointcloud where transformed cloud1 will be outputted
    pcl::PointCloud<PointType> final;
    //Execute the algorithm and output result to "final", that will now hold the transformed cloud
    icp.align(final);

    // Check if algorithm has converged and alignment has been succesful
    if(!icp.hasConverged()) return 0;
    qDebug() << "ICP converged!";


    //Join both pointclouds in final2 for display
    int combinedSize = final.points.size() + cloud2->points.size();
    pcl::PointCloud<PointType> final2;
    final2.resize(combinedSize);

    //Add points from final into final2
    for (int i = 0; i < final.points.size(); ++i) {
        final2.points[i] = final.points[i];
    }
    //Add points from cloud2 into final2
    for (int i = final.points.size(); i < combinedSize; ++i) {
        final2.points[i] = (*cloud2.get()).points[i-final.points.size()];
    }

    qDebug() << final2.points.size();
    qDebug() << "Finished ICP";

    //pcl::io::savePLYFileBinary ("icp.ply" , final);
    qDebug() << "Showing";

   viewer->spinOnce();

   viewer->addPointCloud( final2.makeShared(), "final2" );
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "final2");

   viewer->spinOnce();

    return 0;
}
