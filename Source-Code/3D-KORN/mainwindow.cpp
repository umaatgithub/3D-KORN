#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    tdk_scanWindow(new TDK_ScanWindow(this))
{
    ui->setupUi(this);

    connect(tdk_scanWindow, SIGNAL(mf_SignalDatabasePointCloudUpdated()), ui->centralWidget, SLOT(mf_SlotUpdatePointCloudListTab()));

}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_mf_NewScanToolBar_triggered()
{
    tdk_scanWindow->mv_SensorController->mf_InitializeSensors();
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

}

void MainWindow::on_mf_ActionImportMeshMenuBar_triggered()
{

}

void MainWindow::on_mf_ActionExportPointCloudMenuBar_triggered()
{

}

void MainWindow::on_mf_ActionExportMeshMenuBar_triggered()
{

}
