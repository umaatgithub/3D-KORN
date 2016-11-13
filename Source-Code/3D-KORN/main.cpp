#include "mainwindow.h"
#include <QApplication>

#include "tdk_scanwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.showMaximized();
    w.setMinimumSize(w.size());

//    TDK_ScanWindow tdk;
//    tdk.showMaximized();

    return a.exec();
}
