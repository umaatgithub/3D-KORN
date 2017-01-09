#ifndef TDK_CENTRALWIDGET_H
#define TDK_CENTRALWIDGET_H

#include <QWidget>
#include <QGridLayout>
#include <QDockWidget>
#include <QScrollArea>
#include <QLabel>
#include <QPushButton>

class TDK_CentralWidget : public QWidget
{
    Q_OBJECT
public:
    explicit TDK_CentralWidget(QWidget *parent = 0);

private:
    QGridLayout *layout;

signals:

public slots:
};

#endif // TDK_CENTRALWIDGET_H
