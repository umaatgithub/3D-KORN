#include "tdk_intelr200sensor.h"

//constructor
TDK_IntelR200Sensor::TDK_IntelR200Sensor():TDK_Sensor()
{
    mf_SetMvId(QString("INTELR200"));
    mf_SetMvName(QString("Intel R200"));

    mf_SetupSensor();
    mv_quit = true;

}

//destructor
TDK_IntelR200Sensor::~TDK_IntelR200Sensor()
{
    mv_myManager->Close();
    mv_myManager->Release();
    mv_projection->Release();

}

//returns true if sensor is connected
bool TDK_IntelR200Sensor::mf_IsAvailable()
{
    return false;
}

//stops sensor
bool TDK_IntelR200Sensor::mf_StopSensor()
{
    //release following interfaces : manager, projection
    //also releases device and session

    mv_quit = true;

    return true;
}

//generates a pcl pointcloud from sensor streams
void TDK_IntelR200Sensor::mf_GeneratePointCloud()
{
    // Initialize and Stream Samples
    //mv_myManager->Init();

    //Aligned capture: returns aligned color and depth streams: block/stop return until both streams are ready
    //return NULL pointer if error in acquiring frame
    if (mv_myManager->AcquireFrame(true)<PXC_STATUS_NO_ERROR) {
        qDebug() <<"gutfydy";
        return;
    }

    //mv_myManager->AcquireFrame(true);

    // retrieve the color and depth samples aligned
    mv_alignedImage = mv_myManager->QuerySample();
//qDebug() << mv_alignedImage->IsEmpty();
    // work on color and depth streams of
    mv_colorImage = mv_alignedImage->color;
    mv_depthImage = mv_alignedImage->depth;
    qDebug() << "----------------------";
    // go fetching the next aligned-sample, if required : (this does not 'release' the manager interface)
    mv_myManager->ReleaseFrame();
    qDebug() << ".............................";
    //-----For creating point cloud from depth image (by <projecting> depth image to world coordinates)-------

    //create an ImageData object to store info (in an ImageInfo object) about PXCImage depthImage
    PXCImage::ImageData depthImageData;
    PXCImage::ImageInfo depthImageInfo = mv_depthImage->QueryInfo();
    depthImageData.format = depthImageInfo.format;

    //Initialize planes and pitches arrays, depthImage buffer data starts at ..data.planes[0]
    depthImageData.planes[0] = {0};
    depthImageData.pitches[0] = {0};

    //Acquire read access to depthImage buffer: this is equivalent to getting buffer positions in memory
    //in depthImageData (so that we know where to read)
    mv_depthImage -> AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &depthImageData);

    //short = 2 bytes: each depthImage pixel value is 16 bits long, so we want to traverse the
    //depth buffer in steps of 2 bytes
    short *buffer = (short*)depthImageData.planes[0];
    qDebug() << "Buffered---------------------------";
    //point cloud container for the current request (session)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ( new pcl::PointCloud <pcl::PointXYZRGB> () );

    for (int row = 0; row < mv_depthHeight; row ++){
        for (int col = 0; col < mv_depthWidth; col++){
            PXCPoint3DF32 depthImagePoint;
            PXCPoint3DF32 worldPoint;
            pcl::PointXYZRGB pclworldpoint;

            depthImagePoint.x = col;
            depthImagePoint.y = row;
            depthImagePoint.z = buffer[0];

            mv_projection->ProjectDepthToCamera(1, &depthImagePoint, &worldPoint);

            pclworldpoint.x = (float) worldPoint.x/1000.0;
            pclworldpoint.y = (float) worldPoint.y/1000.0;
            pclworldpoint.z = (float) worldPoint.z/1000.0;

            pclworldpoint.r = (float) 255;
            pclworldpoint.g = (float) 255;
            pclworldpoint.b = (float) 255;

            cloud->points.push_back(pclworldpoint);

            buffer++;
        }
    }

   // mv_myManager->Release();
    qDebug() << "About to set point cloud";
    mf_SetMvPointCloud(cloud);

    qDebug() << "Point cloud set";

}

//keeps updating mv_cloud in the background
void TDK_IntelR200Sensor::mf_threadAcquireCloud()
{
    //pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;
    //cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    qDebug() << "Intel R200 entering thread function";
    while (!mv_quit){
        boost::unique_lock<boost::mutex> lock(mutex);
        qDebug() << "Intel R200 inside thread while";

       mf_GeneratePointCloud();

        //qDebug() << "Intel R200 exiting thread function";
        lock.unlock();
 }
}


//set dimensions of color and depth streams and enable their captures, and initialize other member variables
bool TDK_IntelR200Sensor::mf_SetupSensor ()
{
    qDebug() << "IntelR200 Setup sensor";
    //Notes:
    //Use sensemanager for applications like hand tracking,
    //face tracking, etc., and for organizing and controlling multimodal
    //pipelines; controls = pipeline.{start, stop, pause, getFrame resume}
    //here: mv_myManager provides interface to image acquisition pipeline and the current session and device
    mv_myManager = PXCSenseManager::CreateInstance();

    //Notes:
    //The EnableStream[s] function requests that the specified
    //stream(s) be part of the streaming pipeline. The application
    //can call this function multiple times for different streams.
    //Infrared stream is not available in r200
    mv_colorWidth = 640, mv_colorHeight = 480, mv_fps = 30;
    mv_depthWidth = 320, mv_depthHeight = 240;

    mv_myManager->EnableStream(PXCCapture::STREAM_TYPE_COLOR, mv_colorWidth, mv_colorHeight, mv_fps);
    mv_myManager->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, mv_depthWidth, mv_depthHeight, mv_fps);


     mv_myManager->Init();
    //initialize a pcl pointcloud PointXYZRGB
    //mv_cloud = new pcl::PointCloud <pcl::PointXYZRGB>;

    //Initialize session, capture and device interfaces
    mv_session = mv_myManager->QuerySession();

    PXCSession::ImplDesc desc={};
    desc.group=PXCSession::IMPL_GROUP_SENSOR;
    desc.subgroup=PXCSession::IMPL_SUBGROUP_VIDEO_CAPTURE;

    PXCSession::ImplDesc desc1;
    pxcStatus sts = mv_session->QueryImpl(&desc,0,&desc1);
    sts = mv_session->CreateImpl<PXCCapture>(&desc1,&mv_capture);

    mv_device = mv_capture->CreateDevice(0);

    //initialize the projection interface
    mv_projection = mv_device->CreateProjection();

    qDebug() << "IntelR200 Setup done";
    return true;
}

bool TDK_IntelR200Sensor::mf_StartSensor()
{
    qDebug() << "Starting Intel R200";
    mv_quit = false;


    mv_thread = boost::thread(&TDK_IntelR200Sensor::mf_threadAcquireCloud, this);
    return true;
}
