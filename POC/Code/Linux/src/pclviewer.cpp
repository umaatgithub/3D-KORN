#include "pclviewer.h"
#include "ui_pclviewer.h"
#include <iostream>
//#include <pcl/common/common.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/point_types.h>
//#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/io/vtk_io.h>
#include <vtkDataReader.h>
#include <vtkGenericDataObjectReader.h>
//#include <vtkStructuredGrid.h>
//#include <vtkSmartPointer.h>
//#include <vtkPolyData.h>
#include <string>
//#include <pcl/console/parse.h>
//#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>



//#include <pcl/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <vector>
//#include <vtkSTLWriter.h>
//#include <vtkSTLReader.h>


#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>


#include <pcl/filters/crop_hull.h>

using namespace pcl::visualization;
PCLVisualizer::Ptr viewer;

using namespace pcl;
using namespace std;

PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCLViewer)
{
    ui->setupUi (this);
    this->setWindowTitle ("PCL viewer");

    // Setup the cloud pointer
    cloud.reset ((new PointCloud<PointXYZ>));
    cloud_cropped.reset ((new PointCloud<PointXYZ>));

    cloud_hullx.reset ((new PointCloud<PointXYZ>));

    cloud_hully.reset ((new PointCloud<PointXYZ>));

    cloud_hullz.reset ((new PointCloud<PointXYZ>));

    cloud_randomplane.reset ((new PointCloud<PointXYZ>));


    objects.reset ((new PointCloud<PointXYZ>));
    potatoecloud.reset ((new PointCloud<PointXYZRGBA>));
    // The number of points in the cloud

    //load a file
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/sav/Desktop/chef.pcd" , *cloud);

    // pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/sav/Desktop/canny.pcd" , *cloud);

    /*  pcl::PointIndices::Ptr fInliers (new pcl::PointIndices);
    fInliers = indices of part_of_full_cloud;

    // Extract fInliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract ;
    extract.setInputCloud (full_cloud);
    extract.setIndices (fInliers);
    //extract.setNegative (false); //Removes part_of_cloud but retain the original full_cloud
    extract.setNegative (true); // Removes part_of_cloud from full cloud  and keep the rest
    extract.filter (*full_cloud);
*/

    // Set up the QVTK window
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();

    // Connect "random" button and the function
    connect (ui->pushButton_random,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));

    // Connect R,G,B sliders and their functions
    connect (ui->horizontalSlider_R, SIGNAL (valueChanged (int)), this, SLOT (redSliderValueChanged (int)));
    connect (ui->horizontalSlider_G, SIGNAL (valueChanged (int)), this, SLOT (greenSliderValueChanged (int)));
    connect (ui->horizontalSlider_B, SIGNAL (valueChanged (int)), this, SLOT (blueSliderValueChanged (int)));
    connect (ui->horizontalSlider_R, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
    connect (ui->horizontalSlider_G, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
    connect (ui->horizontalSlider_B, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));

    // Connect point size slider
    connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

    //connect load

    connect (ui->pushButton_load,  SIGNAL (clicked ()), this, SLOT (load ()));
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {

        if (minx > cloud->points[i].x)

            minx = cloud->points[i].x;

        if (maxx < cloud->points[i].x)
            maxx = cloud->points[i].x;


        if (miny > cloud->points[i].y)
            miny = cloud->points[i].y;

        if (maxy < cloud->points[i].y)
            maxy = cloud->points[i].y;

        if (minz > cloud->points[i].z)
            minz = cloud->points[i].z;

        if (maxz < cloud->points[i].z)
            maxz = cloud->points[i].z;

    }

    kx=(maxx-minx)/255+minx;

    ky=(maxy-miny)/255+miny;


    kz=(maxz-minz)/255+minz;


    viewer->addPointCloud (cloud, "cloud");
    //pSliderValueChanged (2);

    viewer->resetCamera ();
    ui->qvtkWidget->update ();
}

void
PCLViewer::randomButtonPressed () //mow callpoisson and stuff
{
    printf ("Random button was pressed\n");

    /*  // Set the new color
    for (size_t i = 0; i < cloud->size(); i++)
    {
        cloud->points[i].r = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
        cloud->points[i].g = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
        cloud->points[i].b = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
    }*/

    ///////////////////////////////////////////////////////////////// http://justpaste.it/code1
    cout<<"start size : " << cloud_cropped->points.size ()<<endl;

    cout << "begin passthrough filter" << endl ;
    PointCloud<PointXYZ>::Ptr filtered(new PointCloud<PointXYZ>());
    PassThrough<PointXYZ> filter;

    filter.setInputCloud(cloud_cropped);
    filter.filter(*filtered);

    cout << "passthrough filter complete" << endl;


    cout<<"after filter size : " << filtered->points.size ()<<endl;

    cout << "begin normal estimation" << endl;
    NormalEstimationOMP<PointXYZ, Normal> ne;
    ne.setNumberOfThreads(8);
    ne.setInputCloud(filtered);
    ne.setRadiusSearch(0.05);
    Eigen::Vector4f centroid;
    compute3DCentroid(*filtered, centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
    ne.compute(*cloud_normals);
    cout << "normal estimation complete" << endl;
    cout << "reverse normals' direction" << endl;

    for(size_t i = 0; i < cloud_normals->size(); ++i){
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }

    cout << "combine points and normals" << endl;
    PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
    concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);


    cout<<"before poisson size : " << cloud->points.size ()<<endl;
    cout << "begin poisson reconstruction" << endl;

    Poisson<PointNormal> poisson;
    poisson.setInputCloud(cloud_smoothed_normals);
    poisson.setDepth(10);
    poisson.setSolverDivide (8);
    poisson.setIsoDivide (8);


    poisson.setConfidence(false);
    poisson.setManifold(true);
    poisson.setOutputPolygons(false);


    // PolygonMesh mesh;
    poisson.reconstruct(mesh);


    io::saveVTKFile("/home/sav/aaa.vtk", mesh);





    cout << "end poisson reconstruction" << endl;



    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/sav/aaa.vtk" , *cloud);



    viewer->updatePointCloud (filtered, "cloud");
    ui->qvtkWidget->update ();
}

void
PCLViewer::RGBsliderReleased ()
{
    /* // Set the new color
    for (size_t i = 0; i < cloud->size (); i++)
    {
        cloud->points[i].r = red;
        cloud->points[i].g = green;
        cloud->points[i].b = blue;
    }
    viewer->updatePointCloud (cloud, "cloud");
    ui->qvtkWidget->update ();*/
}

void
PCLViewer::pSliderValueChanged (int value)
{
    objects->clear();


    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cloud_hullx);
    hull.setDimension(3);
    std::vector<pcl::Vertices> polygons;

    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull (new pcl::PointCloud<pcl::PointXYZ>);
    hull.reconstruct(*surface_hull, polygons);

    for(int i = 0; i < polygons.size(); i++)
        std::cout << polygons[i] << std::endl;

    //pcl::PointCloud<pcl::PointXYZ>::Ptr objects (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropHull<pcl::PointXYZ> bb_filter;

    bb_filter.setDim(2);
    bb_filter.setInputCloud(cloud_randomplane);
    bb_filter.setHullIndices(polygons);
    bb_filter.setHullCloud(cloud_hullx);
    bb_filter.setCropOutside(true);
    bb_filter.filter(*objects);
    std::cout << objects->size() << std::endl;

    // boundingbox_ptr->clear();
    //objects->clear();
    // std::cerr << "PointCloud hull after projection has: "
    //         << cloud_h->points.size () << " data points." << std::endl;
    int size=cloud_cropped->points.size ();
    for (size_t i = 0; i < objects->points.size(); ++i)
    {
        cloud_cropped->push_back(objects->points[i]);

    }

    viewer->updatePointCloud (objects, "cloud");
    ui->qvtkWidget->update ();


    // viewer->addPolygonMesh(output, "polygon");

    /* viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
    ui->qvtkWidget->update ();*/
}
void
PCLViewer::redSliderValueChanged (int value)
{
    red = value;
    // printf ("redSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
    CropPlanesPointsRemoval(red, green,blue,1);

}

void
PCLViewer::greenSliderValueChanged (int value)
{
    green = value;
    //  printf ("greenSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
    //CropBox(red, green,blue);

    CropPlanesPointsRemoval(red, green,blue,2);

}

void
PCLViewer::blueSliderValueChanged (int value)
{
    blue=value;
    //  printf("blueSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
    CropPlanesPointsRemoval(red, green,blue,3);

}
PCLViewer::~PCLViewer ()
{
    delete ui;
}

void PCLViewer::load() //now call meshes and stuff
{
    printf ("Load button was pressed\n");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/sav/Desktop/bunny.pcd" , *cloud_cropped) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

    }

    printf ("Loaded\n");

    ///////////////////////////////////////////////////////////////////// http://stackoverflow.com/questions/12300310/reading-vtk-file
    // simply set filename here (oh static joy)
    std::string inputFilename = "/home/sav/aaa.vtk";
    vtkPolyData* output;
    // Get all data from the file
    vtkSmartPointer<vtkGenericDataObjectReader> reader =
            vtkSmartPointer<vtkGenericDataObjectReader>::New();
    reader->SetFileName(inputFilename.c_str());
    reader->Update();
    if(reader->IsFilePolyData())
    {
        std::cout << "output is a polydata" << std::endl;
        output = reader->GetPolyDataOutput();
        std::cout << "output has " << output->GetNumberOfPoints() << " points." << std::endl;
    }

    //viewer->addModelFromPolyData 	(output);display a vtk file as polygon
    /////////////////////////////////////////////////////

    //prepare the display
    viewer->addPolygonMesh	(mesh);
    //viewer->setShapeRenderingProperties(PCL_VISUALIZER_SHADING,
    //                                  PCL_VISUALIZER_SHADING_PHONG, "polygon");

    std::cout << "Loaded ";
    // viewer->addPolygonMesh(output, "polygon");
    ui->qvtkWidget->update ();
}

void PCLViewer::RandomPlane(int numberofpoints,int randprecision,uint xvalue, uint yvalue,uint zvalue,uint axe){


    cloud_randomplane->clear();


    cloud_randomplane->points.resize(numberofpoints);

    if (axe==0){


        for (int i = 0; i < numberofpoints; i++)
        {

            cloud_randomplane->points[i].z=(rand() % (randprecision+1))*(maxz-minz)/randprecision+minz;
            cloud_randomplane->points[i].y=(rand() % (randprecision+1))*(maxy-miny)/randprecision+miny;
            cloud_randomplane->points[i].x=(xvalue)*(maxx-minx)/255+minx;

        }
    }

    else if (axe==1){

        for (int i = 0; i < numberofpoints; i++)
        {

            cloud_randomplane->points[i].z=(rand() % (randprecision+1))*(maxz-minz)/randprecision+minz;
            cloud_randomplane->points[i].y=(yvalue)*(maxy-miny)/255+miny;
            cloud_randomplane->points[i].x=(rand() % (randprecision+1))*(maxx-minx)/randprecision+minx;

        }
    }

    else {

        for (int i = 0; i < numberofpoints; i++)
        {

            cloud_randomplane->points[i].z=(zvalue)*(maxz-minz)/255+minz;
            cloud_randomplane->points[i].y=(rand() % (randprecision+1))*(maxy-miny)/randprecision+miny;
            cloud_randomplane->points[i].x=(rand() % (randprecision+1))*(maxx-minx)/randprecision+minx;

        }
    }
    /////////////////////////////mesh to cloud


    /*   PointCloud<PointXYZRGBA>::Ptr cloud (new PointCloud<PointXYZRGBA> ());
    pcl::PolygonMesh triangles;
    pcl::io::loadPolygonFilePLY(argv[1], triangles);
    pcl::fromROSMsg(triangles.cloud, *cloud);*/

    ////////////////////////////////////////////




    // cout<< "minz " << minz << "maxZ : "<< maxz ;

}


void PCLViewer::CropPlanesPointsRemoval(uint xi, uint yi, uint zi,uint axe){


    cloud_cropped->clear();
    cloud_hullx->clear();

    cloud_hully->clear();

    cloud_hullz->clear();

    double u=0;
    croppedcloudsize=0;
    double a=0;
    double b=0;
    double c=0;
    cloud_hullx->resize(cloud->size());

    cloud_hully->resize(cloud->size());

    cloud_hullz->resize(cloud->size());

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud_cropped->resize(croppedcloudsize+1);

        if (cloud->points[i].z >= (zi*(maxz-minz)/255)+minz && cloud->points[i].y >= (yi*(maxy-miny)/255)+miny && cloud->points[i].x >= (xi*(maxx-minx)/255)+minx ) { // && cloud->points[i].y <=! y && cloud->points[i].z <=! z){

            cloud_cropped->points[croppedcloudsize]=cloud->points[i];
            croppedcloudsize++;

            if ( (cloud->points[i].x <= ((xi+2)*(maxx-minx)/255)+minx) && (cloud->points[i].x >= ((xi)*(maxx-minx)/255)+minx) ){

                cloud_hullx->points[a]=cloud->points[i];
                cloud_hullx->points[a].x=xi*(maxx-minx)/255+minx;
                a++;

            }

            if ( (cloud->points[i].y <= ((yi+2)*(maxy-miny)/255)+miny) && (cloud->points[i].y >= ((yi)*(maxy-miny)/255)+miny) ){

                cloud_hully->points[b]=cloud->points[i];
                cloud_hully->points[b].y=yi*(maxy-miny)/255+miny;
                b++;

            }

            if ( (cloud->points[i].z <= ((zi+2)*(maxz-minz)/255)+minz) && (cloud->points[i].z >= ((zi)*(maxz-minz)/255)+minz) ){

                cloud_hullz->points[c]=cloud->points[i];
                cloud_hullz->points[c].z=zi*(maxz-minz)/255+minz;
                c++;

            }

        }


    }


    //http://stackoverflow.com/questions/279854/how-do-i-sort-a-vector-of-pairs-based-on-the-second-element-of-the-pair#279878


    cout<<cloud_cropped->points.size ()<<endl;

    RandomPlane(10000,1000,red,green,blue,1);
    FillingCropPlane();

    viewer->updatePointCloud (cloud_cropped, "cloud");
    ui->qvtkWidget->update ();


}

void  PCLViewer::FillingCropPlane (){

    // from https://github.com/PointCloudLibrary/pcl/issues/234

    objects->clear();

    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cloud_hully);
    hull.setDimension(3);
    std::vector<pcl::Vertices> polygons;
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull (new pcl::PointCloud<pcl::PointXYZ>);
    hull.reconstruct(*surface_hull, polygons);

    pcl::CropHull<pcl::PointXYZ> bb_filter;

    bb_filter.setDim(2);
    bb_filter.setInputCloud(cloud_randomplane);
    bb_filter.setHullIndices(polygons);
    bb_filter.setHullCloud(cloud_hully);
    bb_filter.setCropOutside(true);
    bb_filter.filter(*objects);

    for (size_t i = 0; i < objects->points.size(); ++i)
    {
        cloud_cropped->push_back(objects->points[i]);
    }

}





void PCLViewer::on_load_clicked()
{

}
