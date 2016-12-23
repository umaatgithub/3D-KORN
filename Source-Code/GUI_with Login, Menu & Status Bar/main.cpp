#include "mainwindow.h"
#include "tdk_centralwidget.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.showMaximized();
    w.setMinimumSize(w.size());
    w.show();

    return a.exec();
}
