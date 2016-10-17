#include "pclviewer.h"
#include "ui_pclviewer.h"
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/vtk_io.h>
#include <vtkDataReader.h>
#include <vtkGenericDataObjectReader.h>
#include <vtkStructuredGrid.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <string>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vector>
#include <vtkSTLWriter.h>
#include <vtkSTLReader.h>



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
    clouda.reset ((new PointCloud<PointXYZ>));
    // The number of points in the cloud

    //load a file
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/sav/Desktop/chef.pcd" , *cloud);


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
    pSliderValueChanged (2);

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
    cout<<"start size : " << clouda->points.size ()<<endl;

    cout << "begin passthrough filter" << endl ;
    PointCloud<PointXYZ>::Ptr filtered(new PointCloud<PointXYZ>());
    PassThrough<PointXYZ> filter;

    filter.setInputCloud(clouda);
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
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
    ui->qvtkWidget->update ();
}
void
PCLViewer::redSliderValueChanged (int value)
{
    red = value;
    // printf ("redSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
    CropBoxPointsRemoval(red, green,blue,1);
}

void
PCLViewer::greenSliderValueChanged (int value)
{
    green = value;
    //  printf ("greenSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
    //CropBox(red, green,blue);

    CropBoxPointsRemoval(red, green,blue,2);
}

void
PCLViewer::blueSliderValueChanged (int value)
{
    blue=value;
    //  printf("blueSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
    CropBoxPointsRemoval(red, green,blue,3);
}
PCLViewer::~PCLViewer ()
{
    delete ui;
}

void PCLViewer::load() //now call meshes and stuff
{
    printf ("Load button was pressed\n");

    pcl::PointCloud<pcl::PointXYZ>::Ptr clouda (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/sav/Desktop/bunny.pcd" , *clouda) == -1) //* load the file
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

void PCLViewer::CropBox(uint xi, uint yi, uint zi,uint axe){


    clouda->empty();


    /////////////////////////////mesh to cloud


    /*   PointCloud<PointXYZRGBA>::Ptr cloud (new PointCloud<PointXYZRGBA> ());
    pcl::PolygonMesh triangles;
    pcl::io::loadPolygonFilePLY(argv[1], triangles);
    pcl::fromROSMsg(triangles.cloud, *cloud);*/

    ////////////////////////////////////////////

    //http://www.pcl-users.org/Access-pcl-PolygonMesh-triangles-data-td4025718.html

    //cout<<"         balab   "<<mesh->polygons[5].vertices[2] <<endl;


    //  cout<<clouda->width<< '  and  ' <<  cloud->width;
    // clouda->points.resize (clouda->width * clouda->height);
    double a=0;


    for (size_t i = 0; i < cloud->points.size (); ++i)
    {


        clouda->points.resize(a+1);
        clouda->points[a]=cloud->points[i];
        if (clouda->points[a].z < (zi*(maxz-minz)/255)+minz ){//clouda->resize(cloud->points.size()+1);

            clouda->points[a].z=(zi*(maxz-minz)/255)+minz;
            //  clouda->points[i].y=(yri*(maxy-miny)/255)+miny;
            //clouda->points[i].x=(xri*(maxx-minx)/255)+minx;
        }

        if (clouda->points[a].y < (yi*(maxy-miny)/255)+miny){//clouda->resize(cloud->points.size()+1);

            //   clouda->points[i].z=(zri*(maxz-minz)/255)+minz;
            clouda->points[a].y=(yi*(maxy-miny)/255)+miny;}
        //  clouda->points[i].x=(xri*(maxx-minx)/255)+minx;}

        if (clouda->points[a].x < (xi*(maxx-minx)/255)+minx){//clouda->resize(cloud->points.size()+1);


            clouda->points[a].x=(xi*(maxx-minx)/255)+minx;}

        a=a+1;
        //  clouda->points[i].z=(zri*(maxz-minz)/255)+minz;
        // clouda->points[i].y=(yri*(maxy-miny)/255)+miny;
        //clouda->points[i].x=(xi*(maxx-minx)/255)+minx;}
    }


    // cout<< "minz " << minz << "maxZ : "<< maxz ;
    viewer->updatePointCloud (clouda, "cloud");
    viewer->resetCamera ();
    ui->qvtkWidget->update ();

}


void PCLViewer::CropBoxPointsRemoval(uint xi, uint yi, uint zi,uint axe){


    clouda->empty();


    PointCloud<PointXY>::Ptr  outliercoordinates(new pcl::PointCloud<pcl::PointXY>);
    //  pcl::PointCloud<pcl::PointXY>::Ptr temp (new pcl::PointCloud<pcl::PointXY>);

    /////////////////////////////mesh to cloud


    /*   PointCloud<PointXYZRGBA>::Ptr cloud (new PointCloud<PointXYZRGBA> ());
    pcl::PolygonMesh triangles;
    pcl::io::loadPolygonFilePLY(argv[1], triangles);
    pcl::fromROSMsg(triangles.cloud, *cloud);*/

    ////////////////////////////////////////////

    //http://www.pcl-users.org/Access-pcl-PolygonMesh-triangles-data-td4025718.html

    //cout<<"         balab   "<<mesh->polygons[5].vertices[2] <<endl;


    //  cout<<clouda->width<< '  and  ' <<  cloud->width;
    // clouda->points.resize (clouda->width * clouda->height);
    double a=0;
    double u=0;

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        clouda->resize(a+1);

        if (cloud->points[i].z >= (zi*(maxz-minz)/255)+minz && cloud->points[i].y >= (yi*(maxy-miny)/255)+miny && cloud->points[i].x >= (xi*(maxx-minx)/255)+minx ) { // && cloud->points[i].y <=! y && cloud->points[i].z <=! z){

            clouda->points[a]=cloud->points[i];
            a=a+1;
            if ( (cloud->points[i].x <= ((xi+1)*(maxx-minx)/255)+minx) && (cloud->points[i].x >= ((xi)*(maxx-minx)/255)+minx) ){
                // for ( (cloud->points[i].x = (xi*(maxx-minx)/255)+minx))
                outliercoordinates->points.resize (u+1);
                outliercoordinates->points[u].x=cloud->points[a].y;

                outliercoordinates->points[u].y=cloud->points[a].z;
                //     cout<<temp->points[u].x;//<<"     "<<temp.y<< endl;

                u+=1;
                //              for (int p=0; p<outliercoordinates.size(); p+=2) {
            }
            //     }
        }

        else {

            clouda->points[a].x=(xi*(maxx-minx)/255)+minx;
            clouda->points[a].z=(zi*(maxz-minz)/255)+minz;
            clouda->points[a].y=(yi*(maxy-miny)/255)+miny;

            a=a+1;
        }
    }

    Sorting(outliercoordinates);

    /////////////////////Buble sorting


    //http://alienryderflex.com/polygon_fill/

    // cout<< "minz " << minz << "maxZ : "<< maxz ;
    viewer->updatePointCloud (clouda, "cloud");
    viewer->resetCamera ();
    ui->qvtkWidget->update ();


}
void  PCLViewer::FillingCrop ( vector<  PointCloud<PointXY> >  &vec ){
    int  i=0;
    PointCloud<PointXY> temp;
    // while (i<vec.size()-1) {
    // PointCloud<PointXY> temp0= vec.at(i) ;
    //  PointCloud<PointXY> temp1= vec.at(i+1) ;
    /*     if ( temp0->points.x>temp1->points.x) {
        swap=vec.points[i];
        vec.at(i)=vec.at(i+1);
        vec.at(i+1)=swap;
        i--; }
    else {
        i++; }}*/
    // }
}

void  PCLViewer::Sorting(   PointCloud<PointXY>::Ptr    &vec ){//Bubble sorting
    int  i=0;
    pcl::PointXY swap ;// (new pcl::PointXY);
    int a=vec->points.size();
    // cout<<vec->points.size()<<endl;
    while (i<(a-1)) {
        if ( vec->points[i].x > vec->points[i+1].x ) {
            swap=vec->points[i];

            vec->points[i]=vec->points[i+1];
            vec->points[i+1]=swap;

            if (i>0){i--;}
        }
        else {
            i++; }
    }

}

void PCLViewer::on_load_clicked()
{

}
