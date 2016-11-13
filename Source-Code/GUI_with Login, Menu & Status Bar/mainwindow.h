#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include "dialog.h"

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
    void on_pushButton_pressed();

    void on_actionNew_Project_triggered();

    void on_pushButton_2_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_Login_clicked();

private:
    Ui::MainWindow *ui;

    //for access new window

  //SecDialog *dialog;
};

#endif // MAINWINDOW_H
