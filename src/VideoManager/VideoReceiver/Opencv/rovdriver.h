#pragma once
#include <QThread>
#include <QMutex>
#include <QQueue>
#include <QPointF>
#include <QtMath>
#include <QDebug>
#include <QVariant>
#include "FirmwarePlugin.h"
#include "mavlink.h"
#include <QUdpSocket>
#include "Joystick.h"
#include <opencv2/core.hpp> // Basic OpenCV structures (cv::Mat)
using namespace cv;

class Vehicle;

// Detection struct
struct ServoPLL
{
    int servo_center = 1500;   // neutral microseconds
    int servo_signal = 1500;   // current output
    float Kp_area = 0.05f;     // proportional gain for area
    float Kp_pos = 0.05f;      // proportional gain for position
    float deadband_area = 500.0f;
    float deadband_pos = 10.0f;

    Rect referenceRect;         // target lock reference
    bool initialized = false;

    void setReference(const Rect& ref);

    int update(const Rect& detected, Size frameSize);

};

class ROVDriver : public QObject {
    Q_OBJECT

public:
    explicit ROVDriver(QObject *parent = nullptr);
    ~ROVDriver();
    void stop();
    bool isConnected() const;
    float computeYaw(float error);
    void sendRC(int throttle = 1500, int steering = 1500);
    void sendROVControl(int forward, int yaw, int vertical, int strafe);
    void arm();
    void disarm();
    void setMode(const QString &mode);
    void followDiver(const QPointF &frameCenter, const QPointF &blobCenter);
    void setTarget(const QString &address, quint16 port);
    void _run();
    bool sendMessageOnLinkThreadSafe(LinkInterface* link, mavlink_message_t message);
    void _sendTimedCommand();
    float computeVisionGain(float distance);
   public slots:
    Q_INVOKABLE void startThrusterTest();
    Q_INVOKABLE void stopThrusterTest();
    // Private members for test state management
   private slots:
    void _runThrusterTestStep();
private:
    struct Command {
        QString type;
        QVariantList params;
    };

    void executeCommand(const Command &cmd);
    void setRCChannel(const QString &name, int value);
    void sendRCOverride();
    void sendRCOverride2(float,float,float,float,quint16,quint16);

private:
    bool running;
    bool connected;
    QMutex mutex;
    QQueue<Command> commandQueue;
    QVector<int> rc_channels;
    QUdpSocket *udpSocket = nullptr;   // <-- added
    QHostAddress rovAddress;
    quint16 rovPort;
    Vehicle *_activeVehicle = nullptr;
    Joystick *joystick=nullptr;

    FirmwarePlugin * _firmwarePlugin = nullptr;
    QTimer* m_movementTimer = nullptr;
    // Store the desired movement vector (-1.0 to 1.0)
    struct {
        float yaw = 0.0f;
        float thrust = 0.0f;
    } m_currentCommand;
    QTimer* m_testTimer = nullptr;
    int m_testState = 0; // 0: Idle, 1: Forward, 2: Backward, 3: Left, 4: Right, 5: Done
    // ... existing members (like mutex) ...
    float _yawError = 0;
    float _yawIntegral = 0;
    float _yawPrevError = 0;
    QElapsedTimer _pidTimer;
    static constexpr float Kp = 0.8f;
    static constexpr float Ki = 0.0f;
    static constexpr float Kd = 0.15f;
    float dt =0.0f;
    float pidYaw=0.0f;
    static constexpr float minGain = 0.2f;
    static constexpr float maxGain = 0.8f;

    float gain =0.0f;
};
