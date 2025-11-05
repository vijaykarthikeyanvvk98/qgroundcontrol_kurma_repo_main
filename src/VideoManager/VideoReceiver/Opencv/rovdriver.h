#pragma once
#include <QThread>
#include <QMutex>
#include <QQueue>
#include <QPointF>
#include <QtMath>
#include <QDebug>
#include <QVariant>
#include "mavlink.h"
#include <QUdpSocket>

class ROVDriver : public QThread {
    Q_OBJECT

public:
    explicit ROVDriver(QObject *parent = nullptr);
    ~ROVDriver();
    void stop();
    bool isConnected() const;

    void sendRC(int throttle = 1500, int steering = 1500);
    void sendROVControl(int forward, int yaw, int vertical, int strafe);
    void arm();
    void disarm();
    void setMode(const QString &mode);
    void followDiver(const QPointF &frameCenter, const QPointF &blobCenter);
    void setTarget(const QString &address, quint16 port);

protected:
    void run() override;

private:
    struct Command {
        QString type;
        QVariantList params;
    };

    void executeCommand(const Command &cmd);
    void setRCChannel(const QString &name, int value);
    void sendRCOverride();

private:
    bool running;
    bool connected;
    QMutex mutex;
    QQueue<Command> commandQueue;
    QVector<int> rc_channels;
    QUdpSocket *udpSocket = nullptr;   // <-- added
    QHostAddress rovAddress;
    quint16 rovPort;
};
