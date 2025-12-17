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
#include "QGCApplication.h"
#include <QtCore/QSettings>
#include <QtCore/QThread>
#include "LinkInterface.h"
#include "MAVLinkProtocol.h"
#include "QGCCorePlugin.h"
#include "QGCMAVLink.h"
#include "MAVLinkProtocol.h"     // <--- You must have this
#include "MultiVehicleManager.h" // <--- You must have this
#include "Vehicle.h"             // <--- You must have this
LinkInterface*   _link;
ServoPLL pll;
//QGC_LOGGING_CATEGORY(TrackingLog, "qgc.")
// Helper function to get the global MAVLink Protocol instance
// Use the standard QGC singleton access patterns:
MAVLinkProtocol* mavlinkProtocol() {
    return MAVLinkProtocol::instance();
}

MultiVehicleManager* multiVehicleManager() {
    return MultiVehicleManager::instance();
}
ROVDriver::ROVDriver(QObject *parent)
    : running(true), connected(false), rc_channels(8, 1500)
{

}

ROVDriver::~ROVDriver()
{
    stop();
}



bool ROVDriver::sendMessageOnLinkThreadSafe(LinkInterface *link, mavlink_message_t message)
{
    // A. Get the active Vehicle instance
    Vehicle* vehicle = MultiVehicleManager::instance()->activeVehicle();

    if (!link || !vehicle) { // Check both link and vehicle
        qCDebug(VehicleLog) << "sendLinkThreadSafe" << (link ? "link" : "link") << "not connected!";
        return false;
    }

            // B. Fix the type mismatch: Pass the active 'vehicle' pointer, not 'this' (ROVDriver*)
    vehicle->firmwarePlugin()->adjustOutgoingMavlinkMessageThreadSafe(vehicle, link, &message);

            // ... (Rest of the message sending code)
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    int len = mavlink_msg_to_send_buffer(buffer, &message);
    link->writeBytesThreadSafe((const char*)buffer, len);

    return true;
}

void ROVDriver::stop() {
    QMutexLocker locker(&mutex);
    running = false;
    locker.unlock();

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
    QMutexLocker locker(&mutex);

    double dx = blobCenter.x() - frameCenter.x();   // left/right
    double dy = blobCenter.y() - frameCenter.y();   // up/down

    double distance = std::sqrt(dx*dx + dy*dy);
    /*double angle    = qRadiansToDegrees(std::atan2(dy, dx));
    // --- YAW / Steering ---
    int steer = 1500;

    if (angle > -90 && angle < 90) {
        steer = int(1500 + (distance * 1.1));
    } else {
        steer = int(1500 - (distance * 1.1));
    }*/

    //steer = std::clamp(steer, 1000, 2000);

            // --- Throttle ---
    //int thr = int(1500 + (distance * 1.5));
    //thr = std::clamp(thr, 1300, 1700);

            // Python sets throttle = 0 for test, optional:
            // thr = 0;

            // --- Assign outputs ---
    /*int roll     = 1500;      // neutral
    int pitch    = 1500;      // neutral
    int yaw      = steer;
    int throttle = thr;*/

            // Map motions to joystick axes
    int roll     = 1500 + qBound(-300, int(dx * 1.2), 300);       // sway ←→
    int pitch    = 1500 + qBound(-300, int(distance * 0.8), 300);     // surge ↑↓
    int yaw      = 1500 + qBound(-300, int(dx * 0.9), 300);       // rotate
    int throttle = 1500 + qBound(-300, int(-dy * 1.3), 300);      // heave ↑↓

     roll     = std::clamp(roll,     1100, 1900);
     pitch    = std::clamp(pitch,    1100, 1900);
     yaw      = std::clamp(yaw,      1100, 1900);
     throttle = std::clamp(throttle, 1100, 1900);

     //qDebug()<<roll<<pitch;
    sendRCOverride2(roll,pitch,yaw,throttle,0,0);
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
    if (!_link->isConnected()) {
        qCDebug(VehicleLog) << "sendMessageOnLinkThreadSafe" << link << "not connected!";
    }

            // Write message into buffer, prepending start sign
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    _link->writeBytesThreadSafe((const char*)buf, len);
}

void ROVDriver::sendRCOverride2(float roll, float pitch , float yaw, float thrust, quint16 button1, quint16 button2)
{
    Vehicle* vehicle = MultiVehicleManager::instance()->activeVehicle();
    if (!vehicle) {
        qCDebug(VehicleLog) << "No active vehicle found to send MANUAL_CONTROL";
        return;
    }
    // If you absolutely must get the LinkInterface pointer:
    VehicleLinkManager* linkManager = vehicle->vehicleLinkManager();
    SharedLinkInterfacePtr sharedLink = linkManager->primaryLink().lock();
    if (!sharedLink) {
        qCDebug(VehicleLog)<< "sendJoystickDataThreadSafe: primary link gone!";
        return;
    }

    if (sharedLink->linkConfiguration()->isHighLatency()) {
        return;
    }

    mavlink_message_t message;

            // Incoming values are in the range -1:1
    float axesScaling =         1.0 * 1000.0;
    float newRollCommand =      roll * axesScaling;
    float newPitchCommand  =    pitch * axesScaling;    // Joystick data is reverse of mavlink values
    float newYawCommand    =    yaw * axesScaling;
    float newThrustCommand =    thrust * axesScaling;

    mavlink_msg_manual_control_pack_chan(
        static_cast<uint8_t>(MAVLinkProtocol::instance()->getSystemId()),
        static_cast<uint8_t>(MAVLinkProtocol::getComponentId()),
        sharedLink->mavlinkChannel(),
        &message,
        static_cast<uint8_t>(vehicle->id()),
        static_cast<int16_t>(newPitchCommand),
        static_cast<int16_t>(newRollCommand),
        static_cast<int16_t>(newThrustCommand),
        static_cast<int16_t>(newYawCommand),
        button1, button2,
        0,
        0, 0,
        0, 0, 0, 0, 0, 0
        );

    if (sharedLink) {
        //qDebug()<<thrust;
        sendMessageOnLinkThreadSafe(sharedLink.get(),message);
    }
}



void ServoPLL::setReference(const Rect &ref)
{
    referenceRect = ref;
    initialized = true;
}

int ServoPLL::update(const Rect &detected, Size frameSize)
{
    if (!initialized) {
        // Initialize reference in center of frame if not set
        int w = frameSize.width / 4;
        int h = frameSize.height / 3;
        int x = (frameSize.width - w) / 2;
        int y = (frameSize.height - h) / 2;
        referenceRect = Rect(x, y, w, h);
        initialized = true;
    }

            // Compute centers
    float ref_center_x = referenceRect.x + referenceRect.width / 2.0f;
    float det_center_x = detected.x + detected.width / 2.0f;

            // Compute errors
    float area_error = (referenceRect.area() - detected.area());
    float pos_error = (ref_center_x - det_center_x);

            // Deadbands
    if (fabs(area_error) < deadband_area) area_error = 0;
    if (fabs(pos_error) < deadband_pos) pos_error = 0;

            // PLL-like feedback control
    float control_signal = (Kp_area * area_error) + (Kp_pos * pos_error);
    servo_signal = servo_center + static_cast<int>(control_signal);

            // Clamp between servo limits
    servo_signal = std::clamp(servo_signal, 1000, 2000);

            // When locked (errors small), go back to neutral
    if (area_error == 0 && pos_error == 0)
        servo_signal = servo_center;

            //qDebug()<<servo_signal;
    return servo_signal;
}
