#include "WinchSettings.h"

QUdpSocket *sender_socket=nullptr;
WinchSettings::WinchSettings(QObject *parent)
{
    receive_socket = new QUdpSocket(this);
    sender_socket = new QUdpSocket(this);
    receive_socket->bind(QHostAddress::AnyIPv4, 10055);

    connect(receive_socket,&QUdpSocket::readyRead, this, &WinchSettings::processResponse);
    connect(this,&WinchSettings::data_received, this, &WinchSettings::parse_String);

    //qDebug()<<"winch";
}

WinchSettings::~WinchSettings()
{
    receive_socket->flush();
    receive_socket->close();
    receive_socket=nullptr;
    datagram.clear();
    senderAddress= QHostAddress("");
    sender_socket->flush();
    sender_socket->close();
    sender_socket=nullptr;
    receiverAddress= QHostAddress("");

    datagram.clear();
    if(timer && timer->isActive())
    {
        timer->stop();
        timer=nullptr;
    }
}

void WinchSettings::sendDatagram(QByteArray buffer)
{
    QMetaObject::invokeMethod(this, [this, buffer]() {
    if(!senderAddress.isNull())
    {
        sender_socket->writeDatagram(buffer, senderAddress, senderPort);
    }
    }, Qt::QueuedConnection);

}

void WinchSettings::run_motor(int value)
{
    QByteArray buffer;
    buffer.append("@CMD:");
    buffer.append(1);
    buffer.append(":");
    buffer.append(value);
    buffer.append(":");
    buffer.append("#");
    sendDatagram(buffer);
    buffer.clear();
}

void WinchSettings::run_actuator(int value)
{
    QByteArray buffer;
    buffer.append("@CMD:");
    buffer.append(2);
    buffer.append(":");
    buffer.append(value);
    buffer.append(":");
    buffer.append("#");
    sendDatagram(buffer);
    buffer.clear();

}

void WinchSettings::processResponse()
{
    //qDebug()<<"activated";
    /*if (!receive_socket) {
        qDebug() << "socket closed";
        return;
    }*/
    if (receive_socket->state() == QUdpSocket::BoundState) {

        if (receive_socket->hasPendingDatagrams()) {


            datagram.resize(receive_socket->pendingDatagramSize());
            slen = receive_socket->readDatagram(datagram.data(),
                                                datagram.size(),
                                                &senderAddress,
                                                &senderPort);
            if (slen == -1) {
                // break;
            }
            databuffer.append(datagram);

            emit data_received(databuffer);
            datagram.clear();
        }

    }
}

void WinchSettings::parse_String(QByteArray data)
{
    QString text = QString::fromUtf8(data).trimmed();
    QStringList packets = text.split("\r\n", Qt::SkipEmptyParts);

    for (const QString &packet : packets) {
        QString trimmedPacket = packet.trimmed();

        if (trimmedPacket.startsWith("@DATA:") && trimmedPacket.contains(":#")) {
            int start = trimmedPacket.indexOf(":") + 1;
            int end = trimmedPacket.lastIndexOf(":#");

            if (start > 0 && end > start) {
                QString valueStr = trimmedPacket.mid(start, end - start);
                bool ok = false;
                double value = valueStr.toDouble(&ok);
                if (ok) {
                    //qDebug() << "Parsed value:" << value;
                    // Optional: emit data_parsed(value);
                    emit data_to_be_updated(QString::number(value,'f',2));

                } else {
                    //qDebug() << "Invalid number in packet:" << trimmedPacket;
                }
            }
        } else {
            //qDebug() << "Unrecognized packet:" << trimmedPacket;
        }

    }
}
