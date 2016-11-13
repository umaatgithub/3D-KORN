#include "mainwindow.h"
#include "ui_mainwindow.h"
//#include "tdk_scanwindow.h"

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

void MainWindow::on_actionImport_from_scan_triggered()
{
    TDK_ScanWindow *tdk_scanWindow = new TDK_ScanWindow(this);
    tdk_scanWindow->setWindowTitle("3D KORN SCANNER - SCAN WINDOW");
    tdk_scanWindow->showMaximized();
    tdk_scanWindow->setMinimumSize(tdk_scanWindow->size());
//    tdk_scanWindow->update();

}
