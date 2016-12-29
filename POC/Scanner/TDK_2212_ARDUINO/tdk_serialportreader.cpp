#include "tdk_serialportreader.h"
#include <QCoreApplication>

QT_USE_NAMESPACE


TDK_serialPortReader::TDK_serialPortReader(QSerialPort *serialPort, QObject *parent)
    : QObject(parent)
    , m_serialPort(serialPort)
    , m_standardOutput(stdout)
{
    mv_totalAngle=0;
    mv_totalLaps=0;
    connect(m_serialPort, &QSerialPort::readyRead, this, &TDK_serialPortReader::handleReadyRead);
    //connect(m_serialPort, static_cast<void (QSerialPort::*)(QSerialPort::SerialPortError)>(&QSerialPort::error),
    //        this, &TDK_serialPortReader::handleError);
    //connect(&m_timer, &QTimer::timeout, this, &SerialPortReader::handleTimeout);

    //m_timer.start(5000);
}


TDK_serialPortReader::~TDK_serialPortReader()
{
    if (m_serialPort->isOpen())
        m_serialPort->close();
}

//function that sends via serial the command for stopping the turntable
void TDK_serialPortReader::mf_stopPlatform()
{
    int com=0;
    this->mf_sendCommandViaSerial(com);
}

//function that sends via serial the command for setting the number of rotations desired to ddo
void TDK_serialPortReader::mf_setNumberOfRotations(int& num_rot)
{
    int com=num_rot;
    this->mf_sendCommandViaSerial(com);
}

//function that sends via serial the command for starting the turntable
void TDK_serialPortReader::mf_startPlatform()
{
    int com=9;
    this->mf_sendCommandViaSerial(com);
}


//private function that sends a command via serial in an array
void TDK_serialPortReader::mf_sendCommandViaSerial(int& com)
{
    if (m_serialPort->isOpen() && m_serialPort->isWritable())
    {
        QByteArray dayArray;
        dayArray[0]=com;
        m_serialPort->write(dayArray);
        m_serialPort->flush();
        m_serialPort->waitForBytesWritten(2);
    }
    else
    {
        qDebug("Couldn't write in the serial port. Try again!");
    }
}


//////////////////////////////////////////////////////////////////////////////////////////
//Slot function for when the turntable has rotated 5 degrees
void TDK_serialPortReader::handleReadyRead()
{
    static int i=0;
    m_readData.append(m_serialPort->readAll());
    mv_totalAngle+=(int)m_readData[i];
    m_standardOutput << mv_totalAngle<< endl;

    emit(msig_degreesRotated());

    //signal is emitted every time a new rotation is accomplished
    if (mv_totalAngle % 360 == 0)
    {
        mv_totalLaps += 1;
        emit (msig_lapDone());
        m_standardOutput <<("1 complete turn done in the platform. Number of laps: ")<<  mv_totalLaps << endl;

    }

    //Signal is emitted every time we accomplish the step degrees
    if (mv_totalAngle % mv_stepAngle == 0)
    {
        emit (msig_stepDegreesRotated());
    }

    i++;

}

//////////////////////////////////////////////////////////////////////////////////////

/*

//Slot function to handle the timeouts in case a timer is set
void TDK_serialPortReader::handleTimeout()
{
    if (m_readData.isEmpty()) {
        m_standardOutput << QObject::tr("No data was currently available for reading from port %1").arg(m_serialPort->portName()) << endl;
    } else {
        m_standardOutput << QObject::tr("Data successfully received from port %1").arg(m_serialPort->portName()) << endl;
        m_standardOutput << m_readData << endl;
    }


    QCoreApplication::quit();
}
*/

/*
//Slot function to handle a serial port error when the data is being read
void TDK_serialPortReader::handleError(QSerialPort::SerialPortError serialPortError)
{
    if (serialPortError == QSerialPort::ReadError) {
        m_standardOutput << QObject::tr("An I/O error occurred while reading the data from port %1, error: %2").arg(m_serialPort->portName()).arg(m_serialPort->errorString()) << endl;
        QCoreApplication::exit(1);
    }
}

*/
