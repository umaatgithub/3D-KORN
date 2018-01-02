/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include "tdk_centralwidget.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionNew_Scan;
    QAction *actionImportPointCloud;
    QAction *actionImportMesh;
    QAction *actionExportPCD;
    QAction *actionExportPLY;
    QAction *actionExportSTL;
    QAction *actionExportVTK;
    QAction *actionAbout;
    TDK_CentralWidget *centralWidget;
    QMenuBar *menuBar;
    QMenu *menuScan;
    QMenu *menuPointCloud;
    QMenu *menuExport_2;
    QMenu *menuMesh;
    QMenu *menuExport;
    QMenu *menuHelp;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(400, 300);
        actionNew_Scan = new QAction(MainWindow);
        actionNew_Scan->setObjectName(QStringLiteral("actionNew_Scan"));
        actionImportPointCloud = new QAction(MainWindow);
        actionImportPointCloud->setObjectName(QStringLiteral("actionImportPointCloud"));
        actionImportMesh = new QAction(MainWindow);
        actionImportMesh->setObjectName(QStringLiteral("actionImportMesh"));
        actionExportPCD = new QAction(MainWindow);
        actionExportPCD->setObjectName(QStringLiteral("actionExportPCD"));
        actionExportPLY = new QAction(MainWindow);
        actionExportPLY->setObjectName(QStringLiteral("actionExportPLY"));
        actionExportSTL = new QAction(MainWindow);
        actionExportSTL->setObjectName(QStringLiteral("actionExportSTL"));
        actionExportVTK = new QAction(MainWindow);
        actionExportVTK->setObjectName(QStringLiteral("actionExportVTK"));
        actionAbout = new QAction(MainWindow);
        actionAbout->setObjectName(QStringLiteral("actionAbout"));
        centralWidget = new TDK_CentralWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 400, 26));
        menuScan = new QMenu(menuBar);
        menuScan->setObjectName(QStringLiteral("menuScan"));
        menuPointCloud = new QMenu(menuBar);
        menuPointCloud->setObjectName(QStringLiteral("menuPointCloud"));
        menuExport_2 = new QMenu(menuPointCloud);
        menuExport_2->setObjectName(QStringLiteral("menuExport_2"));
        menuMesh = new QMenu(menuBar);
        menuMesh->setObjectName(QStringLiteral("menuMesh"));
        menuExport = new QMenu(menuMesh);
        menuExport->setObjectName(QStringLiteral("menuExport"));
        menuHelp = new QMenu(menuBar);
        menuHelp->setObjectName(QStringLiteral("menuHelp"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuScan->menuAction());
        menuBar->addAction(menuPointCloud->menuAction());
        menuBar->addAction(menuMesh->menuAction());
        menuBar->addAction(menuHelp->menuAction());
        menuScan->addAction(actionNew_Scan);
        menuPointCloud->addAction(actionImportPointCloud);
        menuPointCloud->addAction(menuExport_2->menuAction());
        menuExport_2->addAction(actionExportPCD);
        menuExport_2->addAction(actionExportPLY);
        menuMesh->addAction(actionImportMesh);
        menuMesh->addAction(menuExport->menuAction());
        menuExport->addAction(actionExportSTL);
        menuExport->addAction(actionExportVTK);
        menuHelp->addAction(actionAbout);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "3D KORN SCANNER", Q_NULLPTR));
        actionNew_Scan->setText(QApplication::translate("MainWindow", "New Scan", Q_NULLPTR));
        actionImportPointCloud->setText(QApplication::translate("MainWindow", "Import", Q_NULLPTR));
        actionImportMesh->setText(QApplication::translate("MainWindow", "Import", Q_NULLPTR));
        actionExportPCD->setText(QApplication::translate("MainWindow", "PCD", Q_NULLPTR));
        actionExportPLY->setText(QApplication::translate("MainWindow", "PLY", Q_NULLPTR));
        actionExportSTL->setText(QApplication::translate("MainWindow", "STL", Q_NULLPTR));
        actionExportVTK->setText(QApplication::translate("MainWindow", "VTK", Q_NULLPTR));
        actionAbout->setText(QApplication::translate("MainWindow", "About", Q_NULLPTR));
        menuScan->setTitle(QApplication::translate("MainWindow", "Scan", Q_NULLPTR));
        menuPointCloud->setTitle(QApplication::translate("MainWindow", "PointCloud", Q_NULLPTR));
        menuExport_2->setTitle(QApplication::translate("MainWindow", "Export As", Q_NULLPTR));
        menuMesh->setTitle(QApplication::translate("MainWindow", "Mesh", Q_NULLPTR));
        menuExport->setTitle(QApplication::translate("MainWindow", "Export As", Q_NULLPTR));
        menuHelp->setTitle(QApplication::translate("MainWindow", "Help", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
