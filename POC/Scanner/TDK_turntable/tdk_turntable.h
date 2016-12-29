#ifndef TDK_TURNTABLE_H
#define TDK_TURNTABLE_H

#include <QtSerialPort/QSerialPort>
#include <QTextStream>
#include <QTimer>
#include <QByteArray>
#include <QObject>
#include <QDebug>

/*
QT_USE_NAMESPACE
QT_BEGIN_NAMESPACE
QT_END_NAMESPACE
*/


class TDK_turntable : public QObject
{
    Q_OBJECT

public:
    TDK_turntable(QSerialPort *serialPort, QObject *parent = 0);
    ~TDK_turntable();
    void mf_startPlatform();
    void mf_stopPlatform();
    void mf_setNumberOfRotations(int& num_rot);

signals:
    void msig_lapDone();
    void msig_degreesRotated();
    void msig_stepDegreesRotated();

private slots:
    void handleReadyRead();
    //void handleTimeout();
    //void handleError(QSerialPort::SerialPortError error);

private:
    void mf_sendCommandViaSerial(int &com);
    QSerialPort *m_serialPort;
    QByteArray  m_readData;
    QTextStream m_standardOutput;
    //QTimer      m_timer;
    int         mv_totalAngle;
    int         mv_stepAngle;
    int         mv_totalLaps;
};


#endif // TDK_TURNTABLE_H
