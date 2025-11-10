#include "rovdriver.h"
#include "MavlinkAction.h"
#include "MavlinkActionManager.h"
#include "MavlinkActionsSettings.h"
#include "FirmwarePlugin.h"
#include "GimbalController.h"
#include "MultiVehicleManager.h"
#include "QGCCorePlugin.h"
#include "QGCLoggingCategory.h"
#include "QmlObjectListModel.h"
#include "SettingsManager.h"
#include "Vehicle.h"

#include <QtCore/QSettings>
#include <QtCore/QThread>

/*QGC_LOGGING_CATEGORY(JoystickLog, "qgc.joystick.joystick")
QGC_LOGGING_CATEGORY(JoystickValuesLog, "qgc.joystick.joystickvalues")*/
ROVDriver::ROVDriver(QObject *parent)
    : QThread(parent), running(true), connected(false), rc_channels(8, 1500)
{
   udpSocket = new QUdpSocket(this);
    //QHostAddress rovAddress("192.168.2.1"); // change to your ROV IP
    //quint16 rovPort = 14550;                // default MAVLink UDP port
   // Bind to any local port so we can send/receive datagrams
   rovAddress = QHostAddress("192.168.2.2");
   rovPort = 14550;
   // ✅ Bind the UDP socket so it enters BoundState
   if (!udpSocket->bind(QHostAddress::AnyIPv4, 14551)) {
       qWarning() << "⚠️ Failed to bind UDP socket on port 14551.";
       return;
   }
   if (udpSocket->state() == QUdpSocket::UnconnectedState) {
       //qDebug()<<"unconnected";
       udpSocket->connectToHost(rovAddress,rovPort);
   }


    // Send a test packet to simulate connection
    QByteArray ping = "HELLO_ROV";

   if (udpSocket->state() == QUdpSocket::BoundState) {

    qint64 sent = udpSocket->writeDatagram(ping, rovAddress, rovPort);

    if (sent > 0) {
        connected = true;
        qDebug() << "✅ Sent test ping to ROV at" << rovAddress.toString() << ":" << rovPort;
    } else {
        qWarning() << "⚠️ Failed to send test ping to ROV.";
    }
   }
}

ROVDriver::~ROVDriver()
{
    stop();
    wait(); // <--- Wait for thread to finish before destruction
}

void ROVDriver::run() {
    while (running) {
        if (!connected) {
            qDebug() << "Connecting to vehicle...";
            connected = true;
            qDebug() << "Vehicle connected.";
        }

        mutex.lock();
        if (!commandQueue.isEmpty()) {
            auto cmd = commandQueue.dequeue();
            mutex.unlock();
            executeCommand(cmd);
        } else {
            mutex.unlock();
            msleep(50);
        }
    }
}

void ROVDriver::stop() {
    QMutexLocker locker(&mutex);
    running = false;
    locker.unlock();

    if (isRunning())
        wait();  // Wait for the thread to fully stop

    if (connected) {
        rc_channels.fill(0);
        sendRCOverride();
        connected = false;
    }
}

bool ROVDriver::isConnected() const { return connected; }

void ROVDriver::sendRC(int throttle, int steering) {
    QMutexLocker locker(&mutex);
    commandQueue.enqueue({"rc", QVariantList() << throttle << steering});
}

void ROVDriver::sendROVControl(int forward, int yaw, int vertical, int strafe) {
    QMutexLocker locker(&mutex);
    commandQueue.enqueue({"rovrc", QVariantList() << forward << yaw << vertical << strafe});
}

void ROVDriver::arm() {
    QMutexLocker locker(&mutex);
    commandQueue.enqueue({"arm", QVariantList()});
}

void ROVDriver::disarm() {
    QMutexLocker locker(&mutex);
    commandQueue.enqueue({"disarm", QVariantList()});
}

void ROVDriver::setMode(const QString &mode) {
    QMutexLocker locker(&mutex);
    commandQueue.enqueue({"mode", QVariantList() << mode});
}

void ROVDriver::followDiver(const QPointF &frameCenter, const QPointF &blobCenter) {
    double dx = frameCenter.x() - blobCenter.x();
    double dy = frameCenter.y() - blobCenter.y();
    double distance = std::sqrt(dx * dx + dy * dy);
    double angle = qRadiansToDegrees(std::atan2(dy, dx));

    int steer = 1500;
    if (angle > -90 && angle < 90)
        steer = int(1500 + (distance * 1.1));
    else
        steer = int(1500 - (distance * 1.1));

    int throttle = int(1500 + (distance * 1.5));
    throttle = std::clamp(throttle, 1300, 1700);
    steer = std::clamp(steer, 1000, 2000);

    //qDebug()<<throttle<<steer;
    sendRC(throttle, steer);
    float yaw;
    float pitch;
    float roll;

    //_activeVehicle->sendJoystickDataThreadSafe(roll, pitch, yaw, throttle, 0, 0);

}

void ROVDriver::executeCommand(const Command &cmd) {
    if (cmd.type == "rc") {
        int throttle = cmd.params[0].toInt();
        int steering = cmd.params[1].toInt();
        setRCChannel("throttle", throttle);
        setRCChannel("yaw", steering);
        sendRCOverride();
    } else if (cmd.type == "rovrc") {
        setRCChannel("throttle", cmd.params[0].toInt());
        setRCChannel("yaw", cmd.params[1].toInt());
        setRCChannel("vertical", cmd.params[2].toInt());
        setRCChannel("strafe", cmd.params[3].toInt());
        sendRCOverride();
    } else if (cmd.type == "arm") {
        qDebug() << "Vehicle armed";
    } else if (cmd.type == "disarm") {
        qDebug() << "Vehicle disarmed";
    } else if (cmd.type == "mode") {
        QString mode = cmd.params[0].toString();
        qDebug() << "Mode set to:" << mode;
    }
}

void ROVDriver::setRCChannel(const QString &name, int value) {
    static QMap<QString, int> chanMap = {
        {"roll", 1}, {"pitch", 2}, {"throttle", 3}, {"yaw", 4},
        {"vertical", 5}, {"strafe", 6}
    };

    int idx = chanMap.value(name, -1);
    if (idx >= 1 && idx <= 8)
        rc_channels[idx - 1] = value;
}

void ROVDriver::sendRCOverride() {
    //qDebug() << "RC Override:" << rc_channels;
    if (!udpSocket) {
        qWarning() << "UDP socket not initialized!";
        return;
    }
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint16_t ch1 = rc_channels[0];
    uint16_t ch2 = rc_channels[1];
    uint16_t ch3 = rc_channels[2];
    uint16_t ch4 = rc_channels[3];
    uint16_t ch5 = rc_channels[4];
    uint16_t ch6 = rc_channels[5];
    uint16_t ch7 = rc_channels[6];
    uint16_t ch8 = rc_channels[7];

    // Remaining channels (not used) must be 0 or 65535
    uint16_t ch9  = 0;
    uint16_t ch10 = 0;
    uint16_t ch11 = 0;
    uint16_t ch12 = 0;
    uint16_t ch13 = 0;
    uint16_t ch14 = 0;
    uint16_t ch15 = 0;
    uint16_t ch16 = 0;
    uint16_t ch17 = 0;
    uint16_t ch18 = 0;

    mavlink_msg_rc_channels_override_pack(
        1,      // system_id
        200,    // component_id
        &msg,
        1,      // target_system
        0,      // target_component
        ch1, ch2, ch3, ch4,
        ch5, ch6, ch7, ch8,
        ch9, ch10, ch11, ch12,
        ch13, ch14, ch15, ch16,
        ch17, ch18
        );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    udpSocket->writeDatagram(reinterpret_cast<const char*>(buf), len, rovAddress, rovPort);

}

void ROVDriver::setTarget(const QString &address, quint16 port)
{
    rovAddress = QHostAddress(address);
    rovPort = port;
    qDebug() << "ROV target set to" << rovAddress.toString() << ":" << rovPort;
}
