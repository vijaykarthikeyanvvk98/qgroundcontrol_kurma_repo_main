#ifndef VIDEOSTREAMER_H
#define VIDEOSTREAMER_H

#include <QImage>
#include <QObject>
#include <QThread>
#include "qthread.h"
#include <deque>
#include <opencv2/core.hpp> // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui.hpp>
#include "VideoReceiver.h"
#include <opencv2/dnn.hpp>
#include <iostream>
#include <opencv2/video/tracking.hpp>
#include <vector>
#include <opencv2/highgui.hpp>
#include "rovdriver.h"
using namespace cv;


//extern ROVDriver rov;
// Detection struct
struct ServoPLL {
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

    /*void drawReference(Mat& frame) {
        if (initialized) {
            rectangle(frame, referenceRect, Scalar(0, 255, 255), 2);
            putText(frame, "Reference", Point(referenceRect.x, referenceRect.y - 10),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
        }
    }*/
};

struct Detection {
    int class_id;
    float confidence;
    Rect box;
};
class KalmanBoxTracker
{
   public:
    cv::KalmanFilter kf;
    cv::Mat state;
    KalmanBoxTracker(cv::Rect2f bbox);
    cv::Rect2f predict();
    void update(cv::Rect2f bbox);
};


struct Track
{
    int id;
    cv::Rect2f bbox;
    float conf;
    int last_seen;
    KalmanBoxTracker kalman;
    int hits;
    int age;
    Track(); // <-- ADD THIS LINE
    Track(int tid, cv::Rect2f b, float c, int frame_idx);
};
class IOUTracker
{
   public:
    std::map<int, Track> tracks;
    int next_id = 1;
    float iou_thresh = 0.3f;
    int max_age = 30;

    void update(std::vector<std::pair<cv::Rect2f,float>>& detections, int frame_idx);
    std::vector<Track> get_active_tracks();
};

class VideoStreamer : public VideoReceiver
{
    Q_OBJECT

   public:
    explicit VideoStreamer(QObject *parent = nullptr);
    ~VideoStreamer();


   public:
    void streamVideo();
    void catchFrame(cv::Mat emittedFrame);
    void catchFrame2(cv::Mat emittedFrame);

    void grabImage();
    void grabImage2();
    QRectF m_rectangle;
    void setRectangle(const QRectF &rect);
    void setdetect(bool);
    void rov_follow2(QPointF,QPointF);

    bool isdetected=false;
    IOUTracker iouTracker;
    int frame_idx = 0;
    ROVDriver rov;

    //ROVDriver rov;     // create an instance;
   public slots:
    void openVideoCamera(QString path);
    void streamerThreadSlot();
    void grabFrame();

    void start(uint32_t timeout) override;
    void stop() override;
    void startDecoding(void *sink) override;
    void stopDecoding() override;
    void startRecording(const QString &videoFile, FILE_FORMAT format) override;
    void stopRecording() override;
    void takeScreenshot(const QString &imageFile) override;

   private:
    int FPS_count = 0;
    double fps=30.00;
    //bool is_processing=true;
    QMutex mutex1,mutex2,mutex3,mutex4,mutex5;
    float display_fps=0.0;
    const double Kp_x = 2.0; // horizontal gain
    const double Kp_y = 2.0; // vertical gain
    const double DEADZONE = 10.0;      // pixels
    int servo_center = 1500;  // neutral
    int servo_signal = 1500;  // output
   signals:
    void newImage(QImage &);

    void interrupt_request();

};
class FPSMeter
{
    std::chrono::steady_clock::time_point last;
    std::deque<double> times;
    int maxlen;
   public:
    FPSMeter(int avg_over=30);
    void tick();
    double fps();
};

class Worker: public VideoReceiver
{
    Q_OBJECT
    Q_PROPERTY(QRectF rectangle READ rectangle NOTIFY rectangleChanged)
    Q_PROPERTY(bool detect READ detect NOTIFY detectChanged)
   public:
    explicit Worker(QObject *parent = nullptr);
    ~Worker();
   public:
    QRectF m_rectangle;
    bool isdetected=false;
    QRectF rectangle() const;
    bool detect();
    bool nearlyEqual(float a, float b, float epsilon = 1e-6f);
   public slots:
    void grabImage();
    void grabFrame();
    void start(uint32_t timeout) override;
    void stop() override;
    void startDecoding(void *sink) override;
    void stopDecoding() override;
    void startRecording(const QString &videoFile, FILE_FORMAT format) override;
    void stopRecording() override;
    void takeScreenshot(const QString &imageFile) override;
   signals:
    void emitThreadImage(cv::Mat frameThread);
    void emitThreadImage2(cv::Mat frameThread);
    void rectangleUpdated(const QRectF &rect);
    void rectangleChanged();
    void detectChanged();
    void isdetect(bool);
    void rov_follow(QPointF,QPointF);
   private:

    QMutex mutex1,mutex2,mutex3,mutex4,mutex5;

};

#endif // VIDEOSTREAMER_H
