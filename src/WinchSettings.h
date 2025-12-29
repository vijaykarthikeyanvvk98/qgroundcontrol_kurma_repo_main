#ifndef WINCHSETTINGS_H
#define WINCHSETTINGS_H

#include <QDateTime>
#include <QElapsedTimer>
#include <QUdpSocket>
#include <QtNetwork/QUdpSocket>
#include "qfloat16.h"
#include "qmutex.h"
#include "qtimer.h"
#include <QFile>
#include <QTextStream>

using namespace std;

class WinchSettings:public QObject
{
    Q_OBJECT

   public:
    explicit WinchSettings(QObject *parent = nullptr);
    ~WinchSettings();
   public slots:
    void sendDatagram(QByteArray);
    void run_motor(int);
    void run_actuator(int);
    void processResponse();
    void parse_String(QByteArray);
    void winch_motor_actuator(int,int);
   private:
    QUdpSocket *receive_socket=nullptr;
    QTimer *timer=nullptr;
    int motor_device =1;
    int actuator_device=2;
    int motor_value, actuator_value=0;
    QHostAddress senderAddress=QHostAddress("192.168.2.10");
    quint16 senderPort=10055;
    QHostAddress receiverAddress;
    quint16 receiverPort;
    QByteArray datagram;
    QByteArray databuffer;
    qint64 slen;
   signals:
    void dataupdated();
    void data_received(QByteArray);
    void data_to_be_updated(QString);
};

#endif  // WINCHSETTINGS_H
