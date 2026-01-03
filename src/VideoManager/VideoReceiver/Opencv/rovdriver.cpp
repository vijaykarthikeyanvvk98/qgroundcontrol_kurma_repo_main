# include "rovdriver.h"
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
LinkInterface*   link;
ServoPLL pll;
static int i;
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
    // ... existing initialization code ...
    _pidTimer.start();
    // Initialize the test timer
    //m_testTimer = new QTimer(this);
    //m_testTimer->setInterval(1000); // 1 second per step
    //m_testTimer->setSingleShot(true); // The timer only fires once per state

    // Connect the timer to the state machine slot
    //connect(m_testTimer, &QTimer::timeout, this, &ROVDriver::_runThrusterTestStep);
    //m_testTimer->start();
}

ROVDriver::~ROVDriver()
{
    stop();
    //wait(); // <--- Wait for thread to finish before destruction
}

void ROVDriver::startThrusterTest()
{
    // If the movement timer (used for tracking) is running, stop it first
    // to prevent conflicts, or just ignore the request.
    // Here we'll ensure we don't start two tests.
    if (m_testState != 0) {
        qDebug() << "Thruster test already active or not properly stopped.";
        return;
    }

    m_testState = 1; // Start with the first step (Forward)
    qDebug() << "Thruster Test Started: Step 1 (Forward)";
    _runThrusterTestStep(); // Run the first step immediately
}

void ROVDriver::stopThrusterTest()
{
    if (m_testTimer->isActive()) {
        m_testTimer->stop();
    }
    m_testState = 0; // Set to Idle

    // CRITICAL: Send neutral command immediately to stop thrusters
    sendRCOverride2(0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
    qDebug() << "Thruster Test Stopped (Manual or Completed)";
}
void ROVDriver::_runThrusterTestStep()
{
    // These floats correspond to the inputs for sendRCOverride2(roll, pitch, yaw, thrust, ...)
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float thrust = 0.0f; // Assuming 'thrust' controls forward/backward movement

            // If the timer fires and the state is 5 (Done), stop the test.
    if (m_testState >= 5) {
        stopThrusterTest();
        return;
    }

    // 1. Send the command for the current state (This is the STOP part of the previous command's cycle)
    // The previous state's command was sent 1 second ago. We send the neutral command now.
    sendRCOverride2(0.0f, 0.0f, 0.0f, 0.0f, 0, 0);

    // 2. Prepare the command for the next state
    switch (m_testState) {
        case 1: // Forward
            thrust = 1700.0f;
            qDebug() << "Running: FORWARD (1.0)";
            break;
        case 2: // Backward
            thrust = -1700.0f;
            qDebug() << "Running: BACKWARD (-1.0)";
            break;
        case 3: // Left (We'll use Yaw for steering/turning)
            yaw = -1700.0f;
            qDebug() << "Running: LEFT (Yaw: -1.0)";
            break;
        case 4: // Right (We'll use Yaw for steering/turning)
            yaw = 1700.0f;
            qDebug() << "Running: RIGHT (Yaw: 1.0)";
            break;
        default:
            // If m_testState is 5 (next step), we just stop and return.
            stopThrusterTest();
            return;
    }

            // 3. Send the command for the current state (Start the new 1-second pulse)
    sendRCOverride2(roll, pitch, yaw, thrust, 0, 0);

    // 4. Advance state and restart timer for the next step (1 second later)
    m_testState++;
    m_testTimer->start();
}
void ROVDriver::_sendTimedCommand()
{
    if (m_movementTimer->isActive()) {
        // First timeout: Send the actual movement command
        // Note: The timer is set to SingleShot. It will fire only once.
        // We ensure we send the command before stopping.

        // This is where you send the movement command for 1 second.
        sendRCOverride2(0.0f, 0.0f, m_currentCommand.yaw, m_currentCommand.thrust, 0, 0);

        // To stop the movement, we set the command back to neutral (0, 0)
        // and send the stop command immediately after.
        m_currentCommand.yaw = 0.0f;
        m_currentCommand.thrust = 0.0f;

        // Send the neutral command immediately.
        sendRCOverride2(0.0f, 0.0f, m_currentCommand.yaw, m_currentCommand.thrust, 0, 0);

        // Stop the timer. Since it's SingleShot, it technically stops itself,
        // but explicitly stopping here is cleaner.
        m_movementTimer->stop();
    }
}

float ROVDriver::computeVisionGain(float distance)
{
    constexpr float minGain = 0.2f;
    constexpr float maxGain = 0.8f;

    float gain = maxGain - distance * (maxGain - minGain);
    return qBound(minGain, gain, maxGain);
}
void ROVDriver::_run() {
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
            QThread::msleep(50);
        }
    }
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

float ROVDriver::computeYaw(float error)
{
    dt = _pidTimer.restart() / 1000.0f;


    _yawIntegral += error * dt;
    float derivative = (error - _yawPrevError) / dt;

    _yawPrevError = error;

    float output = Kp * error + Ki * _yawIntegral + Kd * derivative;
    return qBound(-1.0f, output, 1.0f);
}

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
    double halfWidth = frameCenter.x();   // assuming frameCenter.x() = width/2
    float normalizedError = dx / halfWidth; // → [-1, 1]
    normalizedError = std::clamp(normalizedError, -1.0f, 1.0f);
    double distance = std::sqrt(dx*dx + dy*dy);
    double angle    = qRadiansToDegrees(std::atan2(dy, dx));
    // --- YAW / Steering ---
    int steer = 1500;

    if (angle > -90 && angle < 90) {
        steer = int(1500 + (distance * 1.1));
    } else {
        steer = int(1500 - (distance * 1.1));
    }

    steer = std::clamp(steer, 1100, 1900);

            // --- Throttle ---
    int thr = int(1500 + (distance * 1.5));
    thr = std::clamp(thr, 1300, 1700);

            // Python sets throttle = 0 for test, optional:
            // thr = 0;

            // --- Assign outputs ---
    int roll     = 1500;      // neutral
    int pitch    = 1500;      // neutral
    int yaw      = 1600;//steer;
    int throttle = 0;//thr;
    pidYaw = computeYaw(normalizedError);
    //gain = computeVisionGain(distance);
    // Map motions to joystick axes
    /*int roll     = 1500 + qBound(-300, int(dx * 1.2), 300);       // sway ←→
    int pitch    = 1500 + qBound(-300, int(distance * 0.8), 300);     // surge ↑↓
    int yaw      = 1500 + qBound(-300, int(dx * 0.9), 300);       // rotate
    int throttle = 1500 + qBound(-300, int(-dy * 1.3), 300);      // heave ↑↓

    roll     = std::clamp(roll,     1100, 1900);
    pitch    = std::clamp(pitch,    1100, 1900);
    yaw      = std::clamp(yaw,      1100, 1900);
    throttle = std::clamp(throttle, 1100, 1900);*/

    //qDebug()<<yaw;

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
    if (!link->isConnected()) {
        qCDebug(VehicleLog) << "sendMessageOnLinkThreadSafe" << link << "not connected!";
    }

            // Write message into buffer, prepending start sign
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    link->writeBytesThreadSafe((const char*)buf, len);
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
    vehicle->setObjectDetectActive(true);
    //vehicle->setObjectDetectYaw(yaw);
    vehicle->setObjectDetectYaw(pidYaw);

    /*mavlink_message_t message;

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
    }*/
}

void ROVDriver::setTarget(const QString &address, quint16 port)
{
    rovAddress = QHostAddress(address);
    rovPort = port;
    qDebug() << "ROV target set to" << rovAddress.toString() << ":" << rovPort;
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

    return servo_signal;
}
