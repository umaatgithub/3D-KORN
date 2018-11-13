#include "tdk_centralwidget.h"

/***************************************************************************
 * Input argument(s) : QWidget *parent - Parent class pointer
 * Return type       : NA
 * Functionality     : Constructor to initialize variables
 *
 **************************************************************************/
TDK_CentralWidget::TDK_CentralWidget(QWidget *parent) : QWidget(parent),
    mv_CentralGridLayout            (new QGridLayout)                                   ,
    mv_PointCloudQVTKWidget         (new QVTKWidget)                                    ,
    mv_MeshAlgorithmComboBox        (new QComboBox)                                     ,
    mv_GenerateMeshPushButton       (new QPushButton(QString("GENERATE MESH")))         ,
    mv_RegistrationComboBox         (new QComboBox)                                     ,
    mv_RegistrationPushButton       (new QPushButton(QString("REGISTER POINT CLOUDS"))) ,
    mv_PointCloudExplorerTabWidget  (new QTabWidget)                                    ,
    mv_PointCloudListTab            (new QListWidget)                                   ,
    mv_RegisteredPointCloudListTab  (new QListWidget)                                   ,
    mv_MeshListTab                  (new QListWidget)                                   ,
    mv_ScanRegistration             (new TDK_ScanRegistration)                          ,
    mv_numberOfPointCloudsSelected  (0)                                                 ,
    mv_numberOfMeshesSelected       (0)                                                 ,
    mv_2DFeatureDetectionCheckBox(new QCheckBox)
{
    mf_setupUI();

    //Connections for register pointcloud and generate mesh button clicks
    connect(mv_RegistrationPushButton       , SIGNAL(clicked(bool)),
            this                            , SLOT(mf_SlotRegisterPointCloud()));
    connect(mv_GenerateMeshPushButton       , SIGNAL(clicked(bool)),
            this                            , SLOT(mf_SlotGenerateMesh()));

    //Connections for updating registered pointcloud and mesh tabs
    connect(this                            , SIGNAL(mf_SignalRegisteredPointCloudListUpdated()),
            this                            , SLOT(mf_SlotUpdateRegisteredPointCloudListTab()));
    connect(this                            , SIGNAL(mf_SignalMeshListUpdated())                ,
            this                            , SLOT(mf_SlotUpdateMeshListTab()));

    //Connections for updating visualizer display
    connect(mv_PointCloudListTab            , SIGNAL(itemChanged(QListWidgetItem*)),
            this                            , SLOT(mf_SlotUpdatePointCloudDisplay(QListWidgetItem*)));
    connect(mv_RegisteredPointCloudListTab  , SIGNAL(itemChanged(QListWidgetItem*)),
            this                            , SLOT(mf_SlotUpdateRegisteredPointCloudDisplay(QListWidgetItem*)));
    connect(mv_MeshListTab                  , SIGNAL(itemChanged(QListWidgetItem*)),
            this                            , SLOT(mf_SlotUpdateMeshDisplay(QListWidgetItem*)));

    connect(mv_ScanRegistration, SIGNAL(mf_SignalStatusChanged(QString, QColor)),
            this, SLOT(mf_SlotUpdateStatusBar(QString, QColor)));
    connect(mv_2DFeatureDetectionCheckBox, SIGNAL(stateChanged(int)),
            mv_ScanRegistration, SLOT(set_Use2DFeatureDetection(int)));
}

/***************************************************************************
 * Input argument(s) : NA
 * Return type       : NA
 * Functionality     : Destructor to free variables
 *
 **************************************************************************/
TDK_CentralWidget::~TDK_CentralWidget()
{

}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Function to setup central widget ui.
 *
 **************************************************************************/
void TDK_CentralWidget::mf_setupUI()
{
    vtkObject::GlobalWarningDisplayOff();

    mf_SetupPointCloudDisplayWidget();
    mf_SetupPointCloudExplorerTabWidget();
    mf_SetupPointCloudOperationsWidget();

    mv_CentralGridLayout->setColumnMinimumWidth(0, 300);
    mv_CentralGridLayout->setColumnMinimumWidth(1, 300);

    mv_CentralGridLayout->setRowMinimumHeight(0, 300);
    mv_CentralGridLayout->setRowMinimumHeight(1, 300);

    mv_CentralGridLayout->setColumnStretch(1, 1);

    this->setLayout(mv_CentralGridLayout);
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Function to setup pointcloud display widget
 *
 **************************************************************************/
void TDK_CentralWidget::mf_SetupPointCloudDisplayWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Point Cloud Visualizer"));
    dockWidget->setWidget(mv_PointCloudQVTKWidget);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    mv_CentralGridLayout->addWidget(dockWidget, 0, 1, 2, 1);

    mv_PointCloudVisualizer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    mv_PointCloudVisualizer->setBackgroundColor (0.5, 0.5, 0.5);
    mv_PointCloudVisualizer->addCoordinateSystem(1.0);
    mv_PointCloudQVTKWidget->SetRenderWindow ( mv_PointCloudVisualizer->getRenderWindow () );
    mv_PointCloudVisualizer->setupInteractor ( mv_PointCloudQVTKWidget->GetInteractor (), mv_PointCloudQVTKWidget->GetRenderWindow ());
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Function to setup pointcloud explorer widget.
 *
 **************************************************************************/
void TDK_CentralWidget::mf_SetupPointCloudExplorerTabWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Point Cloud Explorer"));
    mv_PointCloudExplorerTabWidget->addTab(mv_PointCloudListTab, QString("PC"));
    mv_PointCloudExplorerTabWidget->addTab(mv_RegisteredPointCloudListTab, QString("REGISTERED PC"));
    mv_PointCloudExplorerTabWidget->addTab(mv_MeshListTab, QString("MESH"));

    dockWidget->setWidget(mv_PointCloudExplorerTabWidget);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    mv_CentralGridLayout->addWidget(dockWidget, 0, 0);
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Function to setup pointcloud operations widget.
 *
 **************************************************************************/
void TDK_CentralWidget::mf_SetupPointCloudOperationsWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Point Cloud Operations"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;

    mv_RegistrationComboBox->setFixedHeight(22);

    mv_RegistrationComboBox->addItem("ICP with normals", "ICP with normals");
    mv_RegistrationComboBox->addItem("ICP", "ICP");

    mv_RegistrationPushButton->setFixedHeight(22);
    mv_RegistrationPushButton->setMinimumWidth(300);

    QFrame* myFrame = new QFrame();
    myFrame->setFrameShape(QFrame::HLine);

    mv_MeshAlgorithmComboBox->setFixedHeight(22);
    mv_MeshAlgorithmComboBox->addItem("Poisson", "Poisson");
    mv_MeshAlgorithmComboBox->addItem("Greedy Triangulation", "Greedy Triangulation");
    mv_MeshAlgorithmComboBox->addItem("Grid Projection", "Grid Projection");
    mv_MeshAlgorithmComboBox->addItem("Marching Cubes", "Marching Cubes");

    mv_GenerateMeshPushButton->setFixedHeight(22);
    mv_GenerateMeshPushButton->setMinimumWidth(300);

    mv_2DFeatureDetectionCheckBox->setFixedHeight(22);
    mv_2DFeatureDetectionCheckBox->setMinimumWidth(300);
    mv_2DFeatureDetectionCheckBox->setText("Use 2D feature matching");

    gridLayout->addWidget(new QLabel("Select registration algorithm : "), 0, 0, 1, 2);
    gridLayout->addWidget(mv_RegistrationComboBox, 0, 2, 1, 2);
    gridLayout->addWidget(mv_RegistrationPushButton, 1, 0, 1, 4);
    gridLayout->addWidget(mv_2DFeatureDetectionCheckBox, 2, 0, 1, 4);
    gridLayout->addWidget(myFrame, 3, 0, 1, 4);
    gridLayout->addWidget(new QLabel("Select mesh algorithm : "), 4, 0, 1, 2);
    gridLayout->addWidget(mv_MeshAlgorithmComboBox, 4, 2, 1, 2);
    gridLayout->addWidget(mv_GenerateMeshPushButton, 5, 0, 1, 4);

    gridLayout->setRowMinimumHeight(0, 30);
    gridLayout->setHorizontalSpacing(10);
    gridLayout->setVerticalSpacing(20);
    gridLayout->setMargin(12);

    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    mv_CentralGridLayout->addWidget(dockWidget, 1, 0);
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Slot function to handle pointcloud registration.
 *
 **************************************************************************/
void TDK_CentralWidget::mf_SlotRegisterPointCloud()
{
    qDebug() << "Check if at least two point clouds selected and run registration";
    if(mv_numberOfPointCloudsSelected > 1){
        for (int i=0, len = mv_PointCloudListTab->count(); i < len; i++){
            if(mv_PointCloudListTab->item(i)->checkState() == Qt::Checked){
                mv_ScanRegistration->addNextPointCloud(TDK_Database::mv_PointCloudsVector[i], 0);
            }
        }
        for (int i=0, len = mv_RegisteredPointCloudListTab->count(); i < len; i++){
            if(mv_RegisteredPointCloudListTab->item(i)->checkState() == Qt::Checked){
                mv_ScanRegistration->addNextPointCloud(TDK_Database::mv_RegisteredPointCloudsVector[i], 0);
            }
        }

        if (mv_2DFeatureDetectionCheckBox->checkState() == Qt::Checked)
           mv_ScanRegistration->mv_use2DFeatureDetection = true;
        else
            mv_ScanRegistration->mv_use2DFeatureDetection = false;

        if (mv_RegistrationComboBox->currentText() == "ICP")
            mv_ScanRegistration->mv_ICP_Normals = false;


        TDK_Database::mf_StaticAddRegisteredPointCloud(mv_ScanRegistration->Process_and_getAlignedPC()->makeShared());
        emit mf_SignalRegisteredPointCloudListUpdated();
    }
    else
    {
        QMessageBox::warning(this, QString("u2.cloud"), QString("Please select at least two point clouds from explorer widget to register."));
    }
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Slot function to handle generated mesh.
 *
 **************************************************************************/
void TDK_CentralWidget::mf_SlotGenerateMesh()
{
    qDebug() << "Check if at least one point cloud is selected and run mesh";
    if(mv_numberOfPointCloudsSelected > 0){
        for (int i=0, len = mv_PointCloudListTab->count(); i < len; i++){
            if(mv_PointCloudListTab->item(i)->checkState() == Qt::Checked){
                mf_SlotUpdateStatusBar(tr("Meshing started..."), QColor(Qt::red));
                pcl::PolygonMesh::Ptr meshPtr ( new PolygonMesh );
                pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud ( new pcl::PointCloud<pcl::PointXYZ> ());
                TDK_Meshing::mf_ConvertFromXYZRGBtoXYZ(TDK_Database::mv_PointCloudsVector[i]->makeShared(), pointcloud);

                if(mv_MeshAlgorithmComboBox->currentText() == "Poisson"){TDK_Meshing::mf_Poisson(pointcloud, meshPtr);}
                else if (mv_MeshAlgorithmComboBox->currentText() == "Greedy Triangulation"){TDK_Meshing::mf_Greedy_Projection_Triangulation(pointcloud, meshPtr);}

                else if (mv_MeshAlgorithmComboBox->currentText() == "Grid Projection"){TDK_Meshing::mf_Grid_Projection(pointcloud,meshPtr);}
                else if (mv_MeshAlgorithmComboBox->currentText() == "Marching Cubes"){TDK_Meshing::mf_Marching_Cubes(pointcloud,meshPtr);}

                TDK_Database::mf_StaticAddMesh(meshPtr);
                emit mf_SignalMeshListUpdated();
            }
        }
        for (int i=0, len = mv_RegisteredPointCloudListTab->count(); i < len; i++){
            if(mv_RegisteredPointCloudListTab->item(i)->checkState() == Qt::Checked){
                mf_SlotUpdateStatusBar(tr("Meshing started..."), QColor(Qt::red));
                pcl::PolygonMesh::Ptr meshPtr ( new PolygonMesh );
                pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud ( new pcl::PointCloud<pcl::PointXYZ> ());
                TDK_Meshing::mf_ConvertFromXYZRGBtoXYZ(TDK_Database::mv_RegisteredPointCloudsVector[i]->makeShared(), pointcloud);

                if(mv_MeshAlgorithmComboBox->currentText() == "Poisson"){TDK_Meshing::mf_Poisson(pointcloud, meshPtr);}
                else if (mv_MeshAlgorithmComboBox->currentText() == "Greedy Triangulation"){TDK_Meshing::mf_Greedy_Projection_Triangulation(pointcloud, meshPtr);}

                else if (mv_MeshAlgorithmComboBox->currentText() == "Grid Projection"){TDK_Meshing::mf_Grid_Projection(pointcloud,meshPtr);}
                else if (mv_MeshAlgorithmComboBox->currentText() == "Marching Cubes"){TDK_Meshing::mf_Marching_Cubes(pointcloud,meshPtr);}

                TDK_Database::mf_StaticAddMesh(meshPtr);
                emit mf_SignalMeshListUpdated();
            }
        mf_SlotUpdateStatusBar(tr("Meshing finished"), QColor(Qt::darkGreen));
        }
    }
    else{

        QMessageBox::warning(this, QString("u2.cloud"), QString("Please select at least one point cloud from explorer widget to generate mesh."));

    }
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Slot function to update pointcloud list tab.
 *
 **************************************************************************/
void TDK_CentralWidget::mf_SlotUpdatePointCloudListTab()
{
    QListWidgetItem* item = new QListWidgetItem;
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(Qt::Unchecked);
    item->setText(TDK_Database::mv_PointCloudsName.back());
    mv_PointCloudListTab->addItem(item);
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Slot function to update registered pointcloud
 *                     list tab.
 *
 **************************************************************************/
void TDK_CentralWidget::mf_SlotUpdateRegisteredPointCloudListTab()
{
    QListWidgetItem* item = new QListWidgetItem;
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(Qt::Unchecked);
    item->setText(TDK_Database::mv_RegisteredPointCloudsName.back());
    mv_RegisteredPointCloudListTab->addItem(item);
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Slot function to update mesh list tab.
 *
 **************************************************************************/
void TDK_CentralWidget::mf_SlotUpdateMeshListTab()
{
    QListWidgetItem* item = new QListWidgetItem;
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(Qt::Unchecked);
    item->setText(TDK_Database::mv_MeshesName.back());
    mv_MeshListTab->addItem(item);
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Slot function to visualize pointcloud in display
 *                     on selection in explorer widget.
 *
 **************************************************************************/
void TDK_CentralWidget::mf_SlotUpdatePointCloudDisplay(QListWidgetItem *item)
{
    if(item->checkState() == Qt::Checked){
        qDebug() << item->text() << "Item checked";
        for(int i = 0, len = item->listWidget()->count(); i < len; i++)
        {
            if(item->listWidget()->item(i)->text() == item->text())
            {
                mv_PointCloudVisualizer->addPointCloud(TDK_Database::mv_PointCloudsVector[i] , item->text().toStdString());
                mv_numberOfPointCloudsSelected++;
                break;
            }
        }
    }
    else
    {
        qDebug() << item->text() << "Item unchecked";
        mv_PointCloudVisualizer->removePointCloud(item->text().toStdString());
        mv_numberOfPointCloudsSelected--;
    }
    mv_PointCloudQVTKWidget->update();
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Slot function to visualize registered pointcloud
 *                     in display on selection in explorer widget.
 *
 **************************************************************************/
void TDK_CentralWidget::mf_SlotUpdateRegisteredPointCloudDisplay(QListWidgetItem *item)
{
    if(item->checkState() == Qt::Checked){
        qDebug() << item->text() << "Registered Item checked";
        for(int i = 0, len = item->listWidget()->count(); i < len; i++)
        {
            if(item->listWidget()->item(i)->text() == item->text())
            {
                mv_PointCloudVisualizer->addPointCloud(TDK_Database::mv_RegisteredPointCloudsVector[i] , item->text().toStdString());
                mv_numberOfPointCloudsSelected++;
                break;
            }
        }
    }
    else{
        qDebug() << item->text() << "Registered Item unchecked";
        mv_PointCloudVisualizer->removePointCloud(item->text().toStdString());
        mv_numberOfPointCloudsSelected--;
    }
    mv_PointCloudQVTKWidget->update();
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Slot function to visualize mesh in display
 *                     on selection in explorer widget.
 *
 **************************************************************************/
void TDK_CentralWidget::mf_SlotUpdateMeshDisplay(QListWidgetItem *item)
{
    if(item->checkState() == Qt::Checked){
        qDebug() << item->text() << "Mesh Item checked";
        for(int i = 0, len = item->listWidget()->count(); i < len; i++)
        {
            if(item->listWidget()->item(i)->text() == item->text())
            {
                mv_PointCloudVisualizer->addPolygonMesh(*(TDK_Database::mv_MeshesVector[i]) , item->text().toStdString());
                mv_numberOfMeshesSelected++;
                break;
            }
        }
    }
    else{
        qDebug() << item->text() << "Mesh Item unchecked";
        mv_PointCloudVisualizer->removePolygonMesh(item->text().toStdString());
        mv_numberOfMeshesSelected--;
    }
    mv_PointCloudQVTKWidget->update();
}

void TDK_CentralWidget::mf_SlotUpdateStatusBar(QString status, QColor statusColor)
{
    emit mf_SignalStatusChanged(status, statusColor);
}
