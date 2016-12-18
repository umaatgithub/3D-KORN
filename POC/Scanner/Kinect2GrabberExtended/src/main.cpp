// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include "kinect2_grabber.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <QDebug>
#include <QString>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <string>
typedef pcl::PointXYZRGB PointType;

bool store_next_pointcloud;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);

  if(event.getKeySym() == "space" && event.keyDown()){
    store_next_pointcloud = true;
  }else if(event.getKeySym() == "r"){
    viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0, 0.0, 0.0 );
  }
}

int main( int argc, char* argv[] )
{
    int filenum = 0;
    store_next_pointcloud = false;

    qDebug ( "Go go!" );//<< endl;
    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer( "Point Cloud Viewer") );
    viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0, 0.0, 0.0 );
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

    // Point Cloud
    pcl::PointCloud<PointType>::ConstPtr cloud;
    pcl::PointCloud<PointType>::Ptr cloud_thresholded(new pcl::PointCloud<PointType>);
    cloud_thresholded->empty();

    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;
    boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =
        [&cloud, &mutex]( const pcl::PointCloud<PointType>::ConstPtr& ptr ){
            boost::mutex::scoped_lock lock( mutex );

            /* Point Cloud Processing */

            cloud = ptr->makeShared();
        };

    // Kinect2Grabber
    boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );

    // Start Grabber
    grabber->start();

    while( !viewer->wasStopped() ){
        // Update Viewer
        viewer->spinOnce();

        boost::mutex::scoped_try_lock lock( mutex );
        if( lock.owns_lock() && cloud ){
            // Update Point Cloud

            double numThresholdedPoint = 0;
            for (size_t i = 0; i < cloud.get()->points.size (); ++i){
               if(cloud.get()->points[i].z < 5){ //1 sweet spot
                   numThresholdedPoint++;
                   cloud_thresholded->points.resize(numThresholdedPoint);

                   cloud_thresholded->points[numThresholdedPoint-1] = cloud.get()->points[i];
               }
            }

            if( !viewer->updatePointCloud( cloud_thresholded, "cloud" ) ){
                viewer->addPointCloud( cloud_thresholded, "cloud" );
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "cloud");
            }

            if(store_next_pointcloud){
                std::string filename = "roberto_" + std::to_string(filenum) + ".ply";
                pcl::io::savePLYFileBinary (filename , *cloud_thresholded.get());
                filenum++;
                store_next_pointcloud = false;
            }
        }
    }

    // Stop Grabber
    grabber->stop();

    // Disconnect Callback Function
    if( connection.connected() ){
        connection.disconnect();
    }

    return 0;
}
