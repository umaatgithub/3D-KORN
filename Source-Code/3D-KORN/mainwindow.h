#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <QFileDialog>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>


#include "tdk_scanwindow.h"
#include "tdk_database.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

signals:
    void mf_SignalDatabasePointCloudUpdated();
    void mf_SignalDatabaseMeshUpdated();

private slots:
    //Tool Bar Actions
    void on_mf_NewScanToolBar_triggered();

    //Menu Bar Actions
    void on_mf_ActionNewScanMenuBar_triggered();
    void on_mf_ActionExitMenuBar_triggered();
    void on_mf_ActionAboutMenuBar_triggered();
    void on_mf_ActionImportPointCloudMenuBar_triggered();
    void on_mf_ActionImportMeshMenuBar_triggered();
    void on_mf_ActionExportPointCloudMenuBar_triggered();
    void on_mf_ActionExportMeshMenuBar_triggered();

private:
    Ui::MainWindow *ui;
    TDK_ScanWindow *tdk_scanWindow;
};

#endif // MAINWINDOW_H
