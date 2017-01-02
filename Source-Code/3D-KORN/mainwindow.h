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
    void    mf_SignalDatabasePointCloudUpdated      ();
    void    mf_SignalDatabaseMeshUpdated            ();

private slots:
    void    on_actionNew_Scan_triggered             ();
    void    on_actionImportPointCloud_triggered     ();
    void    on_actionImportMesh_triggered           ();
    void    on_actionExportPCD_triggered            ();
    void    on_actionExportPLY_triggered            ();
    void    on_actionExportSTL_triggered            ();
    void    on_actionExportVTK_triggered            ();
    void    on_actionAbout_triggered                ();

private:
    Ui::MainWindow      *ui;
    TDK_ScanWindow      *mv_ScanWindow;
};

#endif // MAINWINDOW_H
