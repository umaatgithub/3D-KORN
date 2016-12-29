#include "tdk_centralwidget.h"
#include <QDebug>

TDK_CentralWidget::TDK_CentralWidget(QWidget *parent) : QWidget(parent),
    mv_CentralGridLayout(new QGridLayout),
    mv_PointCloudQVTKWidget(new QVTKWidget),
    mv_MeshAlgorithmComboBox(new QComboBox),
    mv_GenerateMeshPushButton(new QPushButton(QString("GENERATE MESH"))),
    mv_RegistrationComboBox(new QComboBox),
    mv_RegistrationPushButton(new QPushButton(QString("REGISTER POINT CLOUDS"))),
    mv_PointCloudExplorerTabWidget(new QTabWidget),
    mv_PointCloudListTab(new QListWidget),
    mv_RegisteredPointCloudListTab(new QListWidget),
    mv_MeshListTab(new QListWidget),
    mv_ScanRegistration(new TDK_ScanRegistration),
    mv_numberOfPointCloudsSelected(0),
    mv_numberOfMeshesSelected(0)
{
    mf_setupUI();

    connect(mv_RegistrationPushButton, SIGNAL(clicked(bool)), this, SLOT(mf_SlotRegisterPointCloud()));
    connect(mv_GenerateMeshPushButton, SIGNAL(clicked(bool)), this, SLOT(mf_SlotGenerateMesh()));

    connect(this, SIGNAL(mf_SignalRegisteredPointCloudListUpdated()), this, SLOT(mf_SlotUpdateRegisteredPointCloudListTab()));
    connect(this, SIGNAL(mf_SignalMeshListUpdated()), this, SLOT(mf_SlotUpdateMeshListTab()));

    connect(mv_PointCloudListTab, SIGNAL(itemChanged(QListWidgetItem*)), this, SLOT(mf_SlotUpdatePointCloudDisplay(QListWidgetItem*)));
    connect(mv_RegisteredPointCloudListTab, SIGNAL(itemChanged(QListWidgetItem*)), this, SLOT(mf_SlotUpdateRegisteredPointCloudDisplay(QListWidgetItem*)));
    connect(mv_MeshListTab, SIGNAL(itemChanged(QListWidgetItem*)), this, SLOT(mf_SlotUpdateMeshDisplay(QListWidgetItem*)));

}

TDK_CentralWidget::~TDK_CentralWidget()
{

}

void TDK_CentralWidget::mf_setupUI()
{
    vtkObject::GlobalWarningDisplayOff();

    mf_SetupPointCloudDisplayWidget();
    mf_SetupCropWidget();
    mf_SetupInformationWidget();
    mf_SetupPointCloudExplorerTabWidget();
    mf_SetupPointCloudOperationsWidget();

    mv_CentralGridLayout->setColumnMinimumWidth(0, 300);
    mv_CentralGridLayout->setColumnMinimumWidth(1, 300);
    mv_CentralGridLayout->setColumnMinimumWidth(2, 300);

    mv_CentralGridLayout->setRowMinimumHeight(0, 300);
    mv_CentralGridLayout->setRowMinimumHeight(1, 300);

    mv_CentralGridLayout->setColumnStretch(1, 1);

    this->setLayout(mv_CentralGridLayout);
}

void TDK_CentralWidget::mf_SetupPointCloudDisplayWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Point Cloud Visualizer"));
    dockWidget->setWidget(mv_PointCloudQVTKWidget);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    mv_CentralGridLayout->addWidget(dockWidget, 0, 1, 2, 1);

    mv_PointCloudVisualizer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    mv_PointCloudVisualizer->setBackgroundColor (0.1, 0.1, 0.1);
    //mv_PointCloudStreamVisualizer->setCameraPosition( 0.0, 0.0, 2.5, 0.0, 0.0, 0.0 );

    mv_PointCloudVisualizer->addCoordinateSystem(1.0);
//    mv_PointCloudVisualizer->addCube(0, 1, 0, 1, 0, 1, 0, 0, 0,"cube");
//    mv_PointCloudVisualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");
//    mv_PointCloudVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, "cube");
//    mv_PointCloudVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.0, 0.0, "cube");
//    mv_PointCloudVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "cube");

    mv_PointCloudQVTKWidget->SetRenderWindow ( mv_PointCloudVisualizer->getRenderWindow () );
    mv_PointCloudVisualizer->setupInteractor ( mv_PointCloudQVTKWidget->GetInteractor (), mv_PointCloudQVTKWidget->GetRenderWindow ());

}

void TDK_CentralWidget::mf_SetupCropWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Crop Widget"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;



    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    mv_CentralGridLayout->addWidget(dockWidget, 1, 0);

}

void TDK_CentralWidget::mf_SetupInformationWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Information"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;

    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    mv_CentralGridLayout->addWidget(dockWidget, 1, 2);
}

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

void TDK_CentralWidget::mf_SetupPointCloudOperationsWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Point Cloud Operations"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;

    mv_RegistrationComboBox->setFixedHeight(22);
    mv_RegistrationComboBox->addItem("SVD + ICP", "SVD");

    mv_RegistrationPushButton->setFixedHeight(22);
    mv_RegistrationPushButton->setMinimumWidth(300);

    QFrame* myFrame = new QFrame();
    myFrame->setFrameShape(QFrame::HLine);

    mv_MeshAlgorithmComboBox->setFixedHeight(22);
    mv_MeshAlgorithmComboBox->addItem("Poisson", "Poisson");

    mv_GenerateMeshPushButton->setFixedHeight(22);
    mv_GenerateMeshPushButton->setMinimumWidth(300);

    gridLayout->addWidget(new QLabel("Select registration algorithm : "), 0, 0, 1, 2);
    gridLayout->addWidget(mv_RegistrationComboBox, 0, 2, 1, 2);
    gridLayout->addWidget(mv_RegistrationPushButton, 1, 0, 1, 4);
    gridLayout->addWidget(myFrame, 2, 0, 1, 4);
    gridLayout->addWidget(new QLabel("Select mesh algorithm : "), 3, 0, 1, 2);
    gridLayout->addWidget(mv_MeshAlgorithmComboBox, 3, 2, 1, 2);
    gridLayout->addWidget(mv_GenerateMeshPushButton, 4, 0, 1, 4);


    gridLayout->setRowMinimumHeight(0, 30);
    gridLayout->setHorizontalSpacing(10);
    gridLayout->setVerticalSpacing(20);
    gridLayout->setMargin(12);

    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    mv_CentralGridLayout->addWidget(dockWidget, 0, 2);
}

void TDK_CentralWidget::mf_SlotRegisterPointCloud()
{
    qDebug() << "Check if atleast two point clouds selected and run registration";
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
        TDK_Database::mf_StaticAddRegisteredPointCloud(mv_ScanRegistration->postProcess_and_getAlignedPC()->makeShared());
        emit mf_SignalRegisteredPointCloudListUpdated();
    }
    else{
        QMessageBox::warning(this, QString("3D-KORN"), QString("Please select atleast two point clouds from explorer widget to register."));
    }
}

void TDK_CentralWidget::mf_SlotGenerateMesh()
{
    qDebug() << "Check if atleast one point cloud is selected and run mesh";
    if(mv_numberOfPointCloudsSelected > 0){
        for (int i=0, len = mv_PointCloudListTab->count(); i < len; i++){
            if(mv_PointCloudListTab->item(i)->checkState() == Qt::Checked){
                //Call mesh function
                //TDK_Database::mf_StaticAddMesh(meshpointer);
                //emit mf_SignalMeshListUpdated();
            }
        }
        for (int i=0, len = mv_RegisteredPointCloudListTab->count(); i < len; i++){
            if(mv_RegisteredPointCloudListTab->item(i)->checkState() == Qt::Checked){
                //Call mesh function
                //TDK_Database::mf_StaticAddMesh(meshpointer);
                //emit mf_SignalMeshListUpdated();
            }
        }
    }
    else{
        QMessageBox::warning(this, QString("3D-KORN"), QString("Please select atleast one point cloud from explorer widget to generate mesh."));
    }

}

void TDK_CentralWidget::mf_SlotUpdatePointCloudListTab()
{
    QListWidgetItem* item = new QListWidgetItem;
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(Qt::Unchecked);
    item->setText(TDK_Database::mv_PointCloudsName[TDK_Database::mv_PointCloudsName.size()-1]);
    mv_PointCloudListTab->addItem(item);
}

void TDK_CentralWidget::mf_SlotUpdateRegisteredPointCloudListTab()
{
    QListWidgetItem* item = new QListWidgetItem;
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(Qt::Unchecked);
    item->setText(TDK_Database::mv_RegisteredPointCloudsName[TDK_Database::mv_RegisteredPointCloudsName.size()-1]);
    mv_RegisteredPointCloudListTab->addItem(item);
}

void TDK_CentralWidget::mf_SlotUpdateMeshListTab()
{
    QListWidgetItem* item = new QListWidgetItem;
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(Qt::Unchecked);
    item->setText(TDK_Database::mv_MeshesName[TDK_Database::mv_MeshesName.size()-1]);
    mv_MeshListTab->addItem(item);
}

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
    else{
        qDebug() << item->text() << "Item unchecked";
        mv_PointCloudVisualizer->removePointCloud(item->text().toStdString());
        mv_numberOfPointCloudsSelected--;
    }
    mv_PointCloudQVTKWidget->update();

}

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
