#ifndef TDK_TURNTABLE_H
#define TDK_TURNTABLE_H

#include <QObject>

class TDK_TurnTable : public QObject
{
    Q_OBJECT
public:
    explicit TDK_TurnTable(QObject *parent = 0);
    ~TDK_TurnTable();

    unsigned int mv_TotalAngleRotated;
    unsigned int mv_NumberOfRotations;

    unsigned int mf_GetCurrentAngle();
    unsigned int mf_GetNumberOfRotations();

signals:

public slots:
};

#endif // TDK_TURNTABLE_H
