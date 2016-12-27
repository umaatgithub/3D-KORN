#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>

#include "tdk_scanwindow.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

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
};

#endif // MAINWINDOW_H
