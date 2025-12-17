#ifndef VIDEOSTREAMER_H
#define VIDEOSTREAMER_H

#include <QImage>
#include <QObject>
#include <QThread>
#include "qthread.h"
#include <deque>
#include <opencv2/core.hpp> // Basic OpenCV structures (cv::Mat)
#include "VideoReceiver.h"
#include <iostream>
#include <opencv2/video/tracking.hpp>
#include <vector>
#include "rovdriver.h"
using namespace cv;



struct Detection
{
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
    Track();
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
    void setRectangle(const QRectF &rect);
    void setdetect(bool);
    void rov_follow2(QPointF,QPointF);

    QRectF m_rectangle;
    bool isdetected=false;

    ROVDriver rov;

   public slots:
    void openVideoCamera(QString path);
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
    QMutex mutex1,mutex2,mutex4,mutex5,g_frameAccessMutex;
    const double Kp_x = 2.0; // horizontal gain
    const double Kp_y = 2.0; // vertical gain
    const double DEADZONE = 10.0;      // pixels
    int servo_center = 1500;  // neutral
    int servo_signal = 1500;  // output

   signals:
    void newImage(QImage &);
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

class Worker: public QObject
{
    Q_OBJECT

   public:
    explicit Worker(QObject *parent = nullptr);
    ~Worker();

   public:

    QRectF m_rectangle;
    IOUTracker iouTracker;
    int frame_idx = 0;
    bool isdetected=false;
    bool nearlyEqual(float a, float b, float epsilon = 1e-6f);
   public slots:
    void grabImage();
    void grabFrame();
   signals:
    void emitThreadImage(cv::Mat frameThread);
    void rectangleUpdated(const QRectF &rect);
    void isdetect(bool);
    void rov_follow(QPointF,QPointF);

   private:
    QMutex mutex2;
};

#endif // VIDEOSTREAMER_H
