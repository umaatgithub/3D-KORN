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
void TDK_PointOperations::mf_FilterPassthrough(const PointCloud<PointXYZ>::Ptr &mv_PointCloudInput, PointCloud<PointXYZ>::Ptr &mv_PointCloudOutput){
    cout << "begin passthrough filter" << endl ;
    PassThrough<PointXYZ> filter;
    filter.setInputCloud(mv_PointCloudInput);
    filter.filter(*mv_PointCloudOutput); //Filtered PointCloud set as OutPut
    cout << "passthrough filter complete" << endl;
}

//With NormalEstimationFunction we normalize the output received from the FilterPassThrough
//mf_NormalEstimation receives as input the filtered PointCloud and gives as output a PointNormal
void TDK_PointOperations::mf_NormalEstimation(PointCloud<PointXYZ>::Ptr &mv_PointCloudInput, PointCloud<pcl::PointNormal>::Ptr &mv_PointNormalOutput){

    cout << "begin normal estimation" << endl;
    NormalEstimationOMP<PointXYZ, Normal> ne;
    ne.setNumberOfThreads(8);
    ne.setInputCloud(mv_PointCloudInput);
    ne.setRadiusSearch(0.1);
    //ne.setRadiusSearch(0.05);
    Eigen::Vector4f centroid;
    compute3DCentroid(*mv_PointCloudInput, centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
    PointCloud<Normal>::Ptr mv_CloudNormals (new PointCloud<Normal>());
    ne.compute(*mv_CloudNormals);
    cout << "normal estimation complete" << endl;

    cout << "reverse normals' direction" << endl;
    for(size_t i = 0; i < mv_CloudNormals->size(); ++i){
        mv_CloudNormals->points[i].normal_x *= -1;
        mv_CloudNormals->points[i].normal_y *= -1;
        mv_CloudNormals->points[i].normal_z *= -1;
    }

    cout << "combine points and normals" << endl;
    concatenateFields(*mv_PointCloudInput, *mv_CloudNormals, *mv_PointNormalOutput);
}


//mf_PoissonMeshes takes as input a PointCloud, applies all the function defined above and gives as output a mesh
void TDK_PointOperations::mf_PoissonMeshes(const PointCloud<PointXYZ>::Ptr &mv_PointCloudInput ,  pcl::PolygonMesh::Ptr &mv_MeshesOutput){

    //PassThrough filter function
   // PointCloud<PointXYZ>::Ptr mv_PointCloudForFiltering  ((new PointCloud<PointXYZ>)) ;
   // TDK_PointOperations::mf_FilterPassthrough(mv_PointCloudInput,mv_PointCloudForFiltering);


   /* filtro nn funziona bene
    * pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);
    // Create the filtering object
     pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
     sor.setInputCloud (mv_PointCloudInput);
     sor.setMeanK (50);
     sor.setStddevMulThresh (1.0);
     sor.filter (*cloud_filtered1);

*/
    PointCloud<PointXYZ>::Ptr mv_PointCloudForFiltering  ((new PointCloud<PointXYZ>)) ;
    TDK_PointOperations::mf_FilterPassthrough(mv_PointCloudInput,mv_PointCloudForFiltering);

   // TDK_PointOperations::FilterPCPassthrough(mv_PointCloudInput,mv_PointCloudForFiltering);











    //voxel filterin
     pcl::PointCloud<PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<PointXYZ> ());
     pcl::VoxelGrid<pcl::PointXYZ> sor;
      sor.setInputCloud (mv_PointCloudForFiltering);
      sor.setLeafSize (0.001f, 0.001f, 0.001f);
       // sor.setLeafSize (0.01f, 0.01f, 0.01f);

      //0.01 il valore migliore
      //sor.setMinimumPointsNumberPerVoxel(3);//inutile peggiora
      sor.filter (*cloud_filtered1);


/*

//Filtering a PointCloud using ModelOutlierRemoval

     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sphere_filtered (new pcl::PointCloud<pcl::PointXYZ>);


     // modelparameter for this sphere:
      // position.x: 0, position.y: 0, position.z:0, radius: 1
      pcl::ModelCoefficients sphere_coeff;
      sphere_coeff.values.resize (4);
        sphere_coeff.values[0] = 0;
        sphere_coeff.values[1] = 0;
        sphere_coeff.values[2] = 0;
        sphere_coeff.values[3] = 1;

        pcl::ModelOutlierRemoval<pcl::PointXYZ> sphere_filter ;
         sphere_filter.setModelCoefficients (sphere_coeff);
         sphere_filter.setThreshold (0.05);
         sphere_filter.setModelType (pcl::SACMODEL_SPHERE);
         sphere_filter.setInputCloud (mv_PointCloudInput);
         sphere_filter.filter (*cloud_sphere_filtered);





*/








/*
         pcl::PointCloud<PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<PointXYZ> ());
         pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
         // build the filter
         outrem.setInputCloud(cloud_filtered1);
         outrem.setRadiusSearch(0.1);
         outrem.setMinNeighborsInRadius (1);




         //outrem.setUserFilterValue(0.8);
         // apply filter
          outrem.filter (*cloud_filtered2);

*/

    /* // *filtraggio downsampling casual
         //pcl::PointCloud<PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<PointXYZ> ());
      float decimate_percent = 80.0/100.0;
      pcl::RandomSample<pcl::PointXYZ> random_sampler;
      random_sampler.setInputCloud(mv_PointCloudInput);
      int num_output_points = (int) (decimate_percent*mv_PointCloudInput->points.size());
      random_sampler.setSample(num_output_points);
      random_sampler.filter(*cloud_filtered1);
*/


    /*condition on point cloud

// build the condition
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

     pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new    pcl::ConditionAnd<pcl::PointXYZ> ());
     range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new   pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
     range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new   pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 8)));

     // build the filter
      pcl::ConditionalRemoval<pcl::PointXYZ> condrem (range_cond);
      condrem.setInputCloud (mv_PointCloudForFiltering);
      condrem.setKeepOrganized(true);
      condrem.filter (*cloud_filtered);


*/

/*


    // Create a KD-Tree
     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      // Output has the PointNormal type in order to store the normals calculated by MLS
      // pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal>);
         pcl::PointCloud<PointXYZ>::Ptr mls_points(new pcl::PointCloud<PointXYZ> ());
      // Init object (second point type is for the normals, even if unused)
       // pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
         //mls.setComputeNormals (true);
         // Set parameters
           mls.setInputCloud (cloud_filtered1);
           mls.setPolynomialFit (true);
           mls.setSearchMethod (tree);
           mls.setSearchRadius (0,03);

           // Reconstruct
             //mls.process (*mls_points);


*/
//*******************************************************************************************************
     // statistical remove
      // Create the filtering object
       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sta;
        sta.setInputCloud (cloud_filtered1);
        //sta.setMeanK (50);
        sta.setMeanK (50);
        sta.setKeepOrganized(true);
        sta.setStddevMulThresh(10);
       // sta.setUserFilterValue(0.2);


        sta.filter (* cloud_sor);

//********************************************************************************************************//
    //Normal Estimation function
    PointCloud<PointNormal>::Ptr mv_PointNormal1(new PointCloud<PointNormal>());
    TDK_PointOperations::mf_NormalEstimation(cloud_filtered1, mv_PointNormal1);
     // pcl::io::savePCDFile ("bun0-mlspro.pcd", *mv_PointNormal1);
 //  TDK_PointOperations::mf_NormalEstimation(cloud_sor, mv_PointNormal1);



//****************************************************************************************************//
    //LLS FUNZIONANTE


    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
     pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new  pcl::PointCloud<pcl::PointNormal>);
     // Init object (second point type is for the normals, even if unused)
       pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
       mls.setComputeNormals (true);
       tree->setInputCloud(cloud_filtered1);


      // mls.setSqrGaussParam(1000);
      // mls.setPolynomialOrder(1000);
       //  mls.setDilationVoxelSize(100);


       // Set parameters
        mls.setInputCloud (cloud_filtered1); //giusto
        mls.setComputeNormals (true);

        //  mls.setInputCloud (cloud_sor);
        //mls.setInputCloud (*mv_PointNormal1);
         mls.setPolynomialFit (true);
         mls.setSearchMethod (tree);
        // mls.setSearchMethod (false);
        // mls.setSearchRadius (5);
       // mls.setDilationIterations(100);

         mls.setSearchRadius (0.05);


          mls.process (*mls_points);
      //  pcl::io::savePCDFile ("alb-mls.pcd", *mls_points); //first save a new pcd file with the mls then reload this last and apply normalization and poisson

          // prova per applica Poisson

//*******************************************************************************************************************//

/*

//TRIANGULIZATION


            pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
            gp3.setSearchRadius (0.8);

          // Set typical values for the parameters
           gp3.setMu (10);
           gp3.setMaximumNearestNeighbors (20);
           gp3.setMaximumSurfaceAngle(M_PI); // 45 degrees
           gp3.setMinimumAngle(M_PI/10); // 10 degrees
           gp3.setMaximumAngle(M_PI); // 120 degrees
           gp3.setNormalConsistency(false);



           // Get result
             gp3.setInputCloud (mls_points);
            //gp3.setSearchMethod (tree2);
             gp3.reconstruct (*mv_MeshesOutput);


*/




/*
    //marching cube
     search::KdTree<PointNormal>::Ptr tree (new search::KdTree<PointNormal>);
     tree->setInputCloud (mv_PointNormal);
     cout << "begin marching cubes reconstruction" << endl;
     MarchingCubesRBF<PointNormal> mc;
     //PolygonMesh::Ptr mv_MeshesOutput1(new PolygonMesh);
    // PolygonMesh:: triangles;

         mc.setInputCloud (mv_PointNormal);
        // mc.setSearchMethod (tree);
         PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
         std::vector< pcl::Vertices >* polygons (new std::vector< pcl::Vertices >);


       //  mc.reconstruct (*mv_PointNormal,*polygons);

         cout << "end marching cubes reconstruction" << endl;
    // Create search tree*
      //pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
      //tree2->setInputCloud (mls_points);*/



/*
      // Initialize objects
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
      //  pcl::PolygonMesh triangles;
        // Set the maximum distance between connected points (maximum edge length)
          gp3.setSearchRadius (0.5);


          // Set typical values for the parameters
           gp3.setMu (10);
           gp3.setMaximumNearestNeighbors (100);
           gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
           gp3.setMinimumAngle(M_PI/18); // 10 degrees
           gp3.setMaximumAngle(M_PI); // 120 degrees
           gp3.setNormalConsistency(false);


           // Get result
             gp3.setInputCloud (mls_points);
             //gp3.setSearchMethod (tree2);
             gp3.reconstruct (*triangles);

*/







/*first ry filtering
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointNormal>);
    // Create the filtering object
      pcl::PassThrough<pcl::PointNormal> pass;
      pass.setInputCloud (mv_PointNormal);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, 1);

       pass.filter (*cloud_filtered);

 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);
 // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (mv_PointCloudForFiltering);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered1);

*/


/*

    // MarchingCubesRBF<PointNormal> mc;

   // search::KdTree<PointNormal>::Ptr tree (new search::KdTree<PointNormal>);
    //tree->setInputCloud (mv_PointNormal);
     cout << "begin marching cubes reconstruction" << endl;
     //MarchingCubes<PointNormal> mc;
     // MarchingCubesRBF<PointNormal>::Ptr mc ((new MarchingCubesRBF<PointNormal> ));

    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    NormalEstimationOMP<PointXYZ, Normal> ne;

     MarchingCubes<PointNormal> *mc;
      mc = new MarchingCubesHoppe<PointNormal> ();
      search::KdTree<PointNormal>::Ptr tree1 (new search::KdTree<PointNormal>);
      tree1->setInputCloud (mv_PointNormal );
      ne.setInputCloud (mv_PointCloudInput );

     //     ne.setSearchMethod (tree1);
          ne.setKSearch (20);
          PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
          ne.compute (*normals);

      PointCloud<PointNormal>::Ptr CubesMarching  ((new PointCloud<PointNormal>)) ;
       cout << "prova" << endl;
      std::vector<Vertices> triangles;
      //mc->setSearchMethod (tree);
       // mc->setIsoLevel (0.8);
        //mc->setGridResolution (100,100,80);
        //mc->setPercentageExtendGrid (100);
      mc->setInputCloud(mv_PointNormal);
      mc->setSearchMethod (tree1);
      mc->reconstruct(*CubesMarching,triangles);

      //mc->reconstruct (*triangles);


                 double leafSize = 0.01;
                  int isoLevel = 3;


                  PolygonMesh mesh;
                 // MarchingCubesGreedy<PointNormal> mc;

                  // Set parameters
                  mc->setInputCloud(mv_PointNormal);
                  mc->setSearchMethod(tree2);
                  mc->setLeafSize(leafSize);
                  mc->setIsoLevel(isoLevel);

                  // Reconstruct
                  mc.reconstruct (mesh);



                   double leafSize = 0.01;
                   int isoLevel = 3;

                   pcl::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
                                tree->setInputCloud(mv_PointCloudForFiltering);



                           /*     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>::Ptr n;
                                               n->setInputCloud(mv_PointCloudForFiltering);
                                               n->setSearchMethod(tree);
                                               n->setKSearch(20);

                                               pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
                                               n.compute(*normals);*/



 /*   //prova del marching cube
        MarchingCubesRBF<PointNormal>::Ptr mc ((new MarchingCubesRBF<PointNormal> ));
        PointCloud<PointNormal>::Ptr CubesMarching  ((new PointCloud<PointNormal>)) ;
        //std::vector<Vertices> triangles;
        mc->setIsoLevel(0.001);
        mc->setGridResolution(50, 50, 50);
        mc->setInputCloud(mv_PointNormal1);
        mc->reconstruct(*CubesMarching);
*/








    //Poisson Function
    Poisson<PointNormal> mv_Poisson;
    //mv_Poisson.setInputCloud(mls_points);
    mv_Poisson.setInputCloud(mv_PointNormal1);
    mv_Poisson.setDepth(10);
    mv_Poisson.setSolverDivide (8);
    mv_Poisson.setIsoDivide (8);
    mv_Poisson.setSamplesPerNode(1);
    mv_Poisson.setConfidence(false);
    mv_Poisson.setManifold(true);
    mv_Poisson.setOutputPolygons(false);
    //PolygonMesh;
    mv_Poisson.reconstruct(*mv_MeshesOutput);
    //PointCloud<PointNormal>::Ptr mv_PointNormal2(new PointCloud<PointNormal>());

    mv_Poisson.getOutputPolygons();
   mv_Poisson.getIndices();




  //*******************************************************************************************
   //






















   /* // Create search tree*
                    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
                    tree2->setInputCloud(mv_PointNormal1);
                    pcl::MarchingCubesRBF<PointNormal> mc1;
                    mc1.setInputCloud(mv_PointNormal1);
                    mc1.setSearchMethod(tree2);
                    mc1.setIsoLevel(0);
                    mc1.setGridResolution(100, 100, 100);
                    mc1.reconstruct(*mv_MeshesOutput);


*/





 //***************************************************************************************************************//
//MARCHING CUBE FUNZIONANTE
/*

  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  //tree2->setInputCloud(mls_points);
   tree2->setInputCloud(mv_PointNormal1);


    //prova del marching cube funziona
     pcl::MarchingCubes<pcl::PointNormal>::Ptr mc  (new pcl::MarchingCubesHoppe<pcl::PointNormal>);
        //MarchingCubesRBF<PointNormal>::Ptr mc ((new MarchingCubesRBF<PointNormal> ));

       // PointCloud<PointNormal>::Ptr CubesMarching  ((new PointCloud<PointNormal>)) ;
        //std::vector<Vertices> triangles;
        mc->setSearchMethod(tree2);
        mc->setIsoLevel(0);

        mc->setGridResolution(50, 50, 50);




        mc->setInputCloud(mls_points);
        mc->reconstruct(*mv_MeshesOutput);*/
   //****************************************************************************
}

static void mf_PoissonMeshesWithConversion(const PointCloud<PointXYZRGB>::Ptr &PointCloudXYZRGB, PolygonMesh::Ptr &mv_MeshesOutput){

    PointCloud<PointXYZ>::Ptr mv_PointCloudInput  ((new PointCloud<PointXYZ>)) ;
    TDK_PointOperations::mf_ConvertFromXYZRGBtoXYZ(PointCloudXYZRGB, mv_PointCloudInput) ;

    //PassThrough filter function
    PointCloud<PointXYZ>::Ptr mv_PointCloudForFiltering  ((new PointCloud<PointXYZ>)) ;
    TDK_PointOperations::mf_FilterPassthrough(mv_PointCloudInput,mv_PointCloudForFiltering);

    //Normal Estimation function
    PointCloud<PointNormal>::Ptr mv_PointNormal(new PointCloud<PointNormal>());
    TDK_PointOperations::mf_NormalEstimation(mv_PointCloudForFiltering, mv_PointNormal);

    //Poisson Function


//prova del marching cube
    MarchingCubesRBF<PointNormal>::Ptr mc ((new MarchingCubesRBF<PointNormal> ));
    PointCloud<PointNormal>::Ptr CubesMarching  ((new PointCloud<PointNormal>)) ;
    std::vector<Vertices> triangles;
    mc->setInputCloud(mv_PointNormal);
    mc->reconstruct(*CubesMarching,triangles);


    Poisson<PointNormal> mv_Poisson;
    mv_Poisson.setInputCloud(CubesMarching);
    mv_Poisson.setDepth(10);
    mv_Poisson.setSolverDivide (8);
    mv_Poisson.setIsoDivide (8);
    mv_Poisson.setConfidence(false);
    mv_Poisson.setManifold(true);
    mv_Poisson.setOutputPolygons(false);
    //PolygonMesh;
    mv_Poisson.reconstruct(*mv_MeshesOutput);




}


void TDK_PointOperations::mf_ConvertFromXYZRGBtoXYZ(const PointCloud<pcl::PointXYZRGB>::Ptr &mv_PointCloudInput, PointCloud<pcl::PointXYZ>::Ptr &mv_PointCloudOutput){
    copyPointCloud(*mv_PointCloudInput, *mv_PointCloudOutput);
}

/*
          *             ,
                               _/^\_
                              <     >
             *                 /.-.\         *
                      *        `/&\`                   *
                              ,@.*;@,
                             /_o.I %_\    *
                *           (`'--:o(_@;
                           /`;--.,__ `')             *
                          ;@`o % O,*`'`&\
                    *    (`'--)_@ ;o %'()\      *
                         /`;--._`''--._O'@;
                        /&*,()~o`;-.,_ `""`)
             *          /`,@ ;+& () o*`;-';\
                       (`""--.,_0 +% @' &()\
                       /-.,_    ``''--....-'`)  *
                  *    /@%;o`:;'--,.__   __.'\
                      ;*,&(); @ % &^;~`"`o;@();         *
                      /(); o^~; & ().o@*&`;&%O\
                jgs   `"="==""==,,,.,="=="==="`
                   __.----.(\-''#####---...___...-----._
                 '`         \)_`"""""`
                         .--' ')
                       o(  )_-\
                         `"""` `


                                 */
