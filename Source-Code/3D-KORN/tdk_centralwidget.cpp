#include "tdk_centralwidget.h"

TDK_CentralWidget::TDK_CentralWidget(QWidget *parent) : QWidget(parent),
    layout(new QGridLayout), filtering_axis (1),  // = y
    color_mode(4)  // = Rainbow
{
    vtkObject::GlobalWarningDisplayOff();


    QDockWidget *dock = new QDockWidget(tr("Dummy"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;


    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dock->setWidget(scrollArea);
    dock->setFeatures(QDockWidget::NoDockWidgetFeatures);

    layout->addWidget(dock, 0, 0);
    layout->addWidget(new QScrollArea(), 1, 0);
    layout->addWidget(new QScrollArea(), 2, 0);



    layout->addWidget(new QScrollArea(), 0, 1, 3, 1);

    layout->addWidget(new QScrollArea(), 0, 2);
    layout->addWidget(new QScrollArea(), 1, 2);
    layout->addWidget(new QScrollArea(), 2, 2);

    layout->setColumnMinimumWidth(0, 300);
    layout->setColumnMinimumWidth(1, 300);
    layout->setColumnMinimumWidth(2, 300);

    layout->setRowMinimumHeight(0, 150);
    layout->setRowMinimumHeight(1, 150);
    layout->setRowMinimumHeight(2, 150);

    layout->setColumnStretch(1, 1);

    setLayout(layout);
}

TDK_CentralWidget::~TDK_CentralWidget()
{

}
