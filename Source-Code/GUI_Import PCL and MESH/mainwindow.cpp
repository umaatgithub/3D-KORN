#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "QDebug"

#include "TDK_LoadSaveFile.h"
#include "vtkPolyDataReader.h"
#include "vtkSTLWriter.h"
#include "vtkTriangleFilter.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "pclviewer.h"
#include <pcl/io/pcd_io.h>
#include <QFileDialog>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>

TDK_LoadSaveFile::TDK_LoadSaveFile()
{

}


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_mf_NewScanToolBar_triggered()
{
    TDK_ScanWindow *tdk_scanWindow = new TDK_ScanWindow(this);
    if(tdk_scanWindow->mv_SensorController->mf_IsSensorAvailable()){
        tdk_scanWindow->mf_setupUI();
        tdk_scanWindow->setWindowTitle("3D KORN SCANNER - SCAN WINDOW");
        tdk_scanWindow->showMaximized();
        tdk_scanWindow->setMinimumSize(tdk_scanWindow->size());
    }
    else{
        bool retryFlag = true;
        while(retryFlag){
            QMessageBox sensorWarningMessageBox;
            sensorWarningMessageBox.setIcon(QMessageBox::Warning);
            sensorWarningMessageBox.setText("No sensor connected. Please connect sensor and click retry.");
            sensorWarningMessageBox.setStandardButtons(QMessageBox::Retry | QMessageBox::Cancel);
            sensorWarningMessageBox.setDefaultButton(QMessageBox::Retry);
            int retryValue = sensorWarningMessageBox.exec();
            switch (retryValue) {
            case QMessageBox::Retry:
                if(tdk_scanWindow->mv_SensorController->mf_IsSensorAvailable()){
                    tdk_scanWindow->mf_setupUI();
                    tdk_scanWindow->setWindowTitle("3D KORN SCANNER - SCAN WINDOW");
                    tdk_scanWindow->showMaximized();
                    tdk_scanWindow->setMinimumSize(tdk_scanWindow->size());
                    retryFlag = false;
                }
                break;
            case QMessageBox::Cancel:
                retryFlag = false;
                break;
            default:
                retryFlag = false;
                break;
            }
        }
    }
}

void MainWindow::on_mf_ActionNewScanMenuBar_triggered()
{
    on_mf_NewScanToolBar_triggered();
}

void MainWindow::on_mf_ActionExitMenuBar_triggered()
{
    this->close();
}

void MainWindow::on_mf_ActionAboutMenuBar_triggered()
{

}

void MainWindow::on_mf_ActionImportPointCloudMenuBar_triggered()
{
    QString filepath = QFileDialog::getOpenFileName(this, QString("Import"), QString(""), QString("pointcloud (*. pcd *.png *.jpg *.bmp)"));

    pcl::io::loadPCDFile<pcl::PointXYZ> (filepath.toStdString(), *load_cloud);

}

void MainWindow::on_mf_ActionImportMeshMenuBar_triggered()
{
    QString filepath = QFileDialog::getOpenFileName(this, QString("Import"), QString(""), QString("pointcloud (*. pcd *.png *.jpg *.bmp)"));


    pcl::io::loadPolygonFileVTK(filepath.toStdString(), *mesh);



}

void MainWindow::on_mf_ActionExportPointCloudMenuBar_triggered()
{
//    QString filepath = QFileDialog::getOpenFileName(this, QString("Export"), QString(""), QString("pointcloud (*. pcd *.png *.jpg *.bmp)"));

//    pcl::io::savePCDFile (filepath.toStdString(), *save_cloud);
    //pcl::io::savePCDFileASCII(pcdFile, save_cloud);  //the same

}

void MainWindow::on_mf_ActionExportMeshMenuBar_triggered()
{

}
