#include "videostreamer.h"
#include <qtimer.h>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QStandardPaths>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "qdebug.h"
#include "qfloat16.h"
#include "qglobal.h"
#include <opencv2/core.hpp> // Basic OpenCV structures (cv::Mat)
#include <opencv2/dnn.hpp>
#include <iostream>
#include <vector>
#include <QtGlobal>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp> // Video write
#include "opencv2/dnn.hpp"
#include "opencv2/tracking.hpp"
#include <opencv2/core.hpp> // Basic OpenCV structures (cv::Mat)
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp> // Video write#include <iostream>
#include <opencv2/cudaobjdetect.hpp>
#include <numeric>

using namespace cv;
using namespace dnn;
using namespace std;
// ----------- CONFIG -------------
const float INPUT_WIDTH = 640.0;
const float INPUT_HEIGHT = 640.0;
const float SCORE_THRESHOLD = 0.25f;
const float NMS_THRESHOLD = 0.45f;
const float CONFIDENCE_THRESHOLD = 0.5f;
const string MODEL_PATH = "best.onnx";
const string VIDEO_PATH = "C:/Users/vijay/Videos/test2.mp4";
double ref_scale = 0.2; // 20% of frame size (tunable)


static int frameCount = 0;
static bool track_confirm=false;
// --------------------------------
cv::Mat frame,newFrame,frame_to_be_processed;
QTimer tUpdate;
QString streaming_path = "";
int calculated_difference=0;
QTimer *timer=nullptr,*timer2=nullptr;
cv::Mat tempFrame;


static cv::VideoCapture cap;
ServoPLL pll;

Net net;
static int delay = 30;
static cv::Ptr<cv::Tracker> tracker;
static  Rect  tracked_box = Rect(10.0,10.0,10.0,10.0) ;         // Tracked bounding box
std::atomic<bool> obj_tracking = false;      // Are we currently tracking?
static float last_confidence = 0.0f;
static int lostFrames = 0;
QMutex trackerMutex;
bool is_box_detected=false;
bool run_detection = false;
QThread *threadStreamer=nullptr;
QThread *threadStreamer2=nullptr;
std::atomic_bool is_processing=false;

// ---------- KalmanBoxTracker Implementation ----------
KalmanBoxTracker::KalmanBoxTracker(Rect2f bbox)
{
    kf = KalmanFilter(8, 4, 0);
    state = Mat::zeros(8,1,CV_32F);
    kf.transitionMatrix = Mat::eye(8,8,CV_32F);
    for(int i=0;i<4;i++) kf.transitionMatrix.at<float>(i,i+4) = 1.0f;
    kf.measurementMatrix = Mat::zeros(4,8,CV_32F);
    for(int i=0;i<4;i++) kf.measurementMatrix.at<float>(i,i) = 1.0f;
    kf.statePost = (Mat_<float>(8,1) << bbox.x, bbox.y, bbox.width, bbox.height, 0,0,0,0);
}

Rect2f KalmanBoxTracker::predict() {
    Mat pred = kf.predict();
    return Rect2f(pred.at<float>(0), pred.at<float>(1),
                  pred.at<float>(2), pred.at<float>(3));
}

void KalmanBoxTracker::update(Rect2f bbox) {
    Mat meas = (Mat_<float>(4,1) << bbox.x, bbox.y, bbox.width, bbox.height);
    kf.correct(meas);
}

// ---------- Track Constructor ----------
Track::Track() :
                 id(-1), bbox(), conf(0.0f), last_seen(0),
                 kalman(cv::Rect2f()), hits(0), age(0)
{}

Track::Track(int tid, Rect2f b, float c, int frame_idx)
    : id(tid), bbox(b), conf(c), last_seen(frame_idx),
      kalman(b), hits(1), age(0) {}

// ---------- IOU function ----------
static float iou(const Rect2f& a, const Rect2f& b) {
    float interArea = (a & b).area();
    float unionArea = a.area() + b.area() - interArea;
    return unionArea > 0 ? interArea / unionArea : 0.0f;
}

// ---------- IOUTracker Implementation ----------
void IOUTracker::update(vector<pair<Rect2f,float>>& detections, int frame_idx) {
    // Predict next positions
    for (auto &[id, tr] : tracks)
    {
        tr.bbox = tr.kalman.predict();
        tr.age++;
    }

    vector<int> unmatched_tracks, unmatched_detections;
    vector<tuple<int,int,float>> matches;

            // Match detections to existing tracks via IOU
    for (int d = 0; d < detections.size(); ++d)
    {
        float best_iou = 0.0f;
        int best_id = -1;
        for (auto &[id, tr] : tracks)
        {
            float iou_val = iou(tr.bbox, detections[d].first);
            if (iou_val > best_iou)
            {
                best_iou = iou_val;
                best_id = id;
            }
        }
        if (best_iou > iou_thresh && best_id != -1)
        {
            matches.emplace_back(best_id, d, best_iou);
        }
        else
        {
            unmatched_detections.push_back(d);
        }
    }

            // Update matched tracks
    for (auto &[tid, did, iou_val] : matches)
    {
        auto &tr = tracks[tid];
        tr.kalman.update(detections[did].first);
        tr.bbox = detections[did].first;
        tr.conf = detections[did].second;
        tr.last_seen = frame_idx;
        tr.hits++;
    }

            // Create new tracks for unmatched detections
    for (int d : unmatched_detections)
    {
        tracks.emplace(next_id, Track(next_id, detections[d].first, detections[d].second, frame_idx));
        next_id++;
    }

            // Remove old tracks
    for (auto it = tracks.begin(); it != tracks.end(); )
    {
        if (frame_idx - it->second.last_seen > max_age)
            it = tracks.erase(it);
        else
            ++it;
    }}

vector<Track> IOUTracker::get_active_tracks() {
    vector<Track> out;
    for(auto& [id,t] : tracks) out.push_back(t);
    return out;
}

// ---------- FPSMeter Implementation ----------
FPSMeter::FPSMeter(int avg_over) : maxlen(avg_over) {}

void FPSMeter::tick() {
    auto now = chrono::steady_clock::now();
    if(last.time_since_epoch().count()!=0) {
        double dt = chrono::duration<double>(now-last).count();
        times.push_back(dt);
        if(times.size()>maxlen) times.pop_front();
    }
    last = now;
}

double FPSMeter::fps() {
    if(times.empty()) return 0.0;
    double avg = std::accumulate(times.begin(), times.end(), 0.0, [](double a, double b){ return a + b; }) / times.size();
    return avg>0 ? 1.0/avg : 0.0;
}

// ---------- VideoStreamer Implementation ----------

VideoStreamer::VideoStreamer(QObject *parent)
{
    // set target for ROV
    //rov.setTarget("192.168.2.2", 14550);
    //rov.start();
    // Initialize with a default rectangle from C++
    m_rectangle = QRectF(0.0, 0.0, 10.0, 10.0);
    threadStreamer  = new QThread();
    threadStreamer2 = new QThread();




    connect(&tUpdate, &QTimer::timeout, this, &VideoStreamer::streamVideo);
    timer = new QTimer(this);
    timer2 = new QTimer(this);
    net = readNetFromONNX(MODEL_PATH);
    net.setPreferableBackend(DNN_BACKEND_OPENCV);
    net.setPreferableTarget(DNN_TARGET_CPU);
}

VideoStreamer::~VideoStreamer()
{
    if(threadStreamer && threadStreamer->isRunning())
    {
        threadStreamer->requestInterruption();  // Signal worker loop to stop
        threadStreamer->quit();
        threadStreamer->wait();
    }
    if(threadStreamer2 && threadStreamer2->isRunning())
    {
        threadStreamer2->requestInterruption();  // Signal worker loop to stop
        threadStreamer2->quit();
        threadStreamer2->wait();
    }
    if(cap.isOpened())
        cap.release();


    rov.stop();
    rov.wait();

}

void VideoStreamer::streamVideo()
{
        if (!frame.empty()) {
            QImage img = QImage(frame.data, frame.cols, frame.rows, QImage::Format_RGB888).rgbSwapped();
            emit newImage(img);
            track_confirm = track_mode;
        } else {
        }
}

void VideoStreamer::catchFrame(cv::Mat emittedFrame)
{
    QMutexLocker locker(&mutex1);
    emittedFrame.copyTo(frame);

    if (is_processing)
        return;

    emittedFrame.copyTo(frame_to_be_processed);
}

void VideoStreamer::catchFrame2(cv::Mat emittedFrame)
{

    QMutexLocker locker(&mutex3);
    emittedFrame.copyTo(frame);
}

void VideoStreamer::grabImage()
{
    QMutexLocker locker(&mutex2);

    if(frame_to_be_processed.data)
    {
        Mat blob,frameCopy;
        Mat output;

        float x_factor=0.0,y_factor=0.0,x_center=0.0,y_center=0.0,w=0.0,h=0.0,conf=0.0;

        vector<Mat> outputs;
        // Apply Non-Maximum Suppression
        vector<int> indices;
        vector<Rect> boxes;
        vector<float> confidences;

        int rows=0;

        float box_conf=0.0;

        frame_to_be_processed.copyTo(frameCopy);

        is_processing=true;
        //is_box_detected=false;
        frameCount++;

        if (frameCount % 10 == 0 || !obj_tracking)               // No tracker yet
        {
            blobFromImage(frameCopy, blob, 1.0 / 255.0, Size(INPUT_WIDTH, INPUT_HEIGHT), Scalar(), true, false);
            net.setInput(blob);
            net.forward(outputs, net.getUnconnectedOutLayersNames());

            output = outputs[0];  // [1, 5, 8400]
            rows = output.size[2];

            x_factor = (float)frameCopy.size().width / INPUT_WIDTH;
            y_factor = (float)frameCopy.size().height / INPUT_HEIGHT;

            for (int i = 0; i < rows; ++i)
            {
                x_center = output.at<float>(0, 0, i);
                y_center = output.at<float>(0, 1, i);
                w = output.at<float>(0, 2, i);
                h = output.at<float>(0, 3, i);
                conf = output.at<float>(0, 4, i);

                if (conf >= CONFIDENCE_THRESHOLD)
                {

                    int left = int((x_center - 0.5f * w) * x_factor);
                    int top = int((y_center - 0.5f * h) * y_factor);
                    int width = int(w * x_factor);
                    int height = int(h * y_factor);

                    boxes.emplace_back(left, top, width, height);
                    confidences.push_back(conf);
                    //if (width > 0 && height > 0)

                       //dets.emplace_back(Rect2f(left, top, width, height), conf);

                }
            }

                    // Apply IOU-based tracking (keeps same IDs)
                    //iouTracker.update(dets, frame_idx++);

            dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, indices);

            if (!indices.empty())
            {

                // Pick highest confidence box manually (safe)
                int bestIdx = indices[0];
                float bestConf = confidences[bestIdx];
                float bestArea = boxes[indices[0]].area();

                for (size_t i = 1; i < indices.size(); ++i)
                {
                    int idx = indices[i];
                    float conf = confidences[idx];
                    double area = boxes[idx].area();

                            // 2️⃣ If confidence nearly equal, prefer bigger area
                    if (conf > bestConf || fabs(conf - bestConf) < 1e-3f)
                    {
                        if (area > bestArea)
                        {
                            bestConf = conf;
                            bestIdx = idx;
                            bestArea = area;
                        }

                        else if(fabs(area - bestArea) < 1.0f)
                        {
                            if (obj_tracking && !tracked_box.empty())
                            {
                                try
                                {
                                    cv::Point2d newCenter(
                                        boxes[idx].x + boxes[idx].width / 2.0,
                                        boxes[idx].y + boxes[idx].height / 2.0
                                        );

                                    cv::Point2d oldCenter(
                                        tracked_box.x + tracked_box.width / 2.0,
                                        tracked_box.y + tracked_box.height / 2.0
                                        );

                                    double distNew = cv::norm(newCenter - oldCenter);

                                    if (fabs(distNew) > 1e-3f)
                                        continue;// closer to currently tracked box

                                    else
                                    {
                                        bestIdx = idx;
                                        bestArea = area;
                                    }
                                }

                                catch (const cv::Exception& e)
                                {
                                    qDebug() << "⚠️ Proximity comparison failed:" << e.what();
                                }
                            }

                            else
                                continue;
                        }

                        else if(area < bestArea)
                            continue;
                    }

                    else if(conf< bestConf)
                    {
                        if (area > bestArea)
                        {

                            if (obj_tracking && !tracked_box.empty())
                            {
                                try
                                {
                                    cv::Point2d newCenter(
                                        boxes[idx].x + boxes[idx].width / 2.0,
                                        boxes[idx].y + boxes[idx].height / 2.0
                                        );
                                    cv::Point2d oldCenter(
                                        tracked_box.x + tracked_box.width / 2.0,
                                        tracked_box.y + tracked_box.height / 2.0
                                        );
                                    cv::Point2d bestCenter(
                                        boxes[bestIdx].x + boxes[bestIdx].width / 2.0,
                                        boxes[bestIdx].y + boxes[bestIdx].height / 2.0
                                        );

                                    double distNew = cv::norm(newCenter - oldCenter);
                                    double distBest = cv::norm(bestCenter - oldCenter);

                                    if (distNew < distBest)
                                    {
                                        bestIdx = idx; // closer to currently tracked box
                                        bestArea = area;
                                    }

                                    else
                                        continue;
                                }

                                catch (const cv::Exception& e)
                                {
                                    qDebug() << "⚠️ Proximity comparison failed:" << e.what();
                                }
                            }
                        }

                        else if(fabs(area - bestArea) < 1.0f || area < bestArea)
                            continue;
                    }
                }

                tracked_box = boxes[bestIdx];
                is_box_detected = true;

                        // Initialize tracker
                tracker->init(frameCopy, tracked_box);
                obj_tracking = true;
            }

            else
            {
                obj_tracking = false;
                is_box_detected=false;
            }


        }

        else if (obj_tracking)
        {
            // Update tracker
            if (!tracker->update(frameCopy, tracked_box))
            {
                // Tracking failed, force detection next frame
                obj_tracking = false;
                tracked_box = cv::Rect(); // clear stale box
                //emit isdetect(false);
            }

            else
            {
                if(is_box_detected)
                {



                    int servo_signal = pll.update(tracked_box, frameCopy.size());

                    QPointF  frame_center(frameCopy.cols / 2.0, frameCopy.rows / 2.0);
                    QPointF  obj_center(tracked_box.x + tracked_box.width / 2.0,
                                       tracked_box.y + tracked_box.height / 2.0);
                    //rov.followDiver(frame_center, obj_center);

                    /*double offset_x = obj_center.x - frame_center.x; // horizontal
                    double offset_y = obj_center.y - frame_center.y; // vertical
                    double distance = sqrt(offset_x * offset_x + offset_y * offset_y);

                                     double max_x = frameCopy.cols / 2.0;
                                     double max_y = frameCopy.rows / 2.0;

                                      // Yaw: 0-359
                                      double yaw = (abs(offset_x) < DEADZONE) ? 180.0 : 180.0 + (offset_x / max_x) * 180.0;
                                      if (yaw < 0) yaw += 360;
                                      if (yaw >= 360) yaw -= 360;

                                       // Pitch: -90 to +90
                                       double pitch = (abs(offset_y) < DEADZONE) ? 0.0 : -(offset_y / max_y) * 90.0;

                                        // Roll: -180 to +180
                                        double roll = (abs(offset_x) < DEADZONE) ? 0.0 : (offset_x / max_x) * 180.0;

                                        // cout << "Yaw: " << yaw << "  Pitch: " << pitch << "  Roll: " << roll << endl;

                                          auto mapToServo = [](double offset, double maxOffset) {
                                              double mapped = 1500 + (offset / maxOffset) * 500.0;
                                              return std::clamp(mapped, 1000.0, 2000.0);
                                          };


                                           double servo_x = mapToServo(offset_x, max_x); // Left/right
                                           double servo_y = mapToServo(-offset_y, max_y); // Up/down (invert y)


                                            Rect ref_rect(frame_center.x - (frameCopy.cols * ref_scale) / 2.0,
                                                          frame_center.y - (frameCopy.rows * ref_scale) / 2.0,
                                                          frameCopy.cols * ref_scale,
                                                          frameCopy.rows * ref_scale);

                                             double obj_area = tracked_box.area();
                                             double ref_area = ref_rect.area();
                                             double area_ratio = obj_area / ref_area;

                                              QString moveX, moveY, moveZ;

                                               // --- Horizontal (left-right) ---
                                               if (abs(offset_x) > DEADZONE) {
                                                   moveX = (offset_x > 0) ? "RIGHT" : "LEFT";
                                               } else moveX = "CENTER";

                                                // --- Vertical (up-down) ---
                                                if (abs(offset_y) > DEADZONE) {
                                                    moveY = (offset_y > 0) ? "DOWN" : "UP";
                                                } else moveY = "CENTER";

                                                 // --- Depth (forward-backward) ---
                                                 if (area_ratio > 1.1)
                                                     moveZ = "FORWARD";
                                                 else if (area_ratio < 0.9)
                                                     moveZ = "BACKWARD";
                                                 else
                                                     moveZ = "ALIGNED";

                                                  qDebug() << "MoveX:" << moveX
                                                           << "MoveY:" << moveY
                                                           << "MoveZ:" << moveZ
                                                           << "AreaRatio:" << area_ratio
                                                           << "Distance:" << distance;*/
                }
                else;
                    //emit isdetect(false);
            }
        }

        /*else
        {
            // Just predict new positions without YOLO (using Kalman)
                std::vector<std::pair<cv::Rect2f, float>> empty_dets;
            iouTracker.update(empty_dets, frame_idx++);
        }
        auto active_tracks = iouTracker.get_active_tracks();
        if (!active_tracks.empty())
        {
            // You can choose to emit *all* active track boxes
            for (const auto &track : active_tracks)
            {
                QRectF rect(track.bbox.x, track.bbox.y, track.bbox.width, track.bbox.height);
                emit rectangleUpdated(rect);
            }

                     // Optional: mark detection state
                     emit isdetect(true);
                     obj_tracking = true;
                 }
                 else
                 {
                     emit isdetect(false);
                     obj_tracking = false;
                 }*/

    }
    is_processing=false;
}

void VideoStreamer::openVideoCamera(QString path)
{

    /* streaming_path ="udpsrc port=5600 caps=\"application/x-rtp, media=(string)video, encoding-name=(string)H264, payload=96 ! "
         "rtph264depay ! h264parse ! decodebin ! videoconvert ! appsink";// "udp://@:5600?overrun_nonfatal=1&analyzeduration=1000000&buffer_size=65535";
     if (streaming_path.length() == 1)
     {
         if(!cap.isOpened())
             cap.open(streaming_path.toInt());
     }
     else
     {
         if(!cap.isOpened())
             cap.open("udp://@192.168.2.1:5600",cv::CAP_FFMPEG );
     }

       if(!cap.isOpened())
           qDebug()<<"Error";*/
    cap.open("C:/Users/vijay/Videos/test2.mp4");
    setStarted(true);
    emit onStartComplete(STATUS_OK);
    /*cap.open("udpsrc port=5000 ! "
        "application/"
        "x-rtp,encoding-name=H264,payload="
        "96 ! rtph264depay   ! avdec_h264  ! videoconvert ! appsink drop=true sync=false max-buffers=1", cv::CAP_GSTREAMER);*/
    //qDebug()<<streaming_path;
    // Assuming `this` is a QObject-based parent (like MainWindow or Controller)
    /*tracker = cv::TrackerKCF::create();

     cap.read(frame_to_be_processed);
      trackingBox = cv::selectROI(frame_to_be_processed,false);
     tracker->init(frame_to_be_processed,trackingBox);*/
    // 👈 Owned by this
    Worker *worker1= new Worker();
    Worker *worker2= new Worker();
    worker1->moveToThread(threadStreamer);
    worker2->moveToThread(threadStreamer2);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    display_fps = (cap.get(cv::CAP_PROP_FPS))/2;
    if (cap.get(cv::CAP_PROP_FPS) <= 0)
    {
        timer->setInterval(1000/5);
        tUpdate.start(1000 / 5);
        timer2->setInterval(1000/5);

        fps = 25;
        calculated_difference = 1000 / 25;

    }
    else
    {
        timer->setInterval(1000/cap.get(cv::CAP_PROP_FPS));
        timer2->setInterval((1000/cap.get(cv::CAP_PROP_FPS))+10);

        tUpdate.start(1000 /cap.get(cv::CAP_PROP_FPS));
        fps=cap.get(cv::CAP_PROP_FPS);
        calculated_difference = 1000 / cap.get(cv::CAP_PROP_FPS);

    }
    connect(timer, &QTimer::timeout, worker1, &Worker::grabFrame);
    connect(timer2, &QTimer::timeout, worker2, &Worker::grabImage);

    QObject::connect(threadStreamer, SIGNAL(started()), timer, SLOT(start()));
    QObject::connect(threadStreamer2, SIGNAL(started()), timer2, SLOT(start()));
    QObject::connect(worker1, &Worker::emitThreadImage, this, &VideoStreamer::catchFrame);
    QObject::connect(worker2, &Worker::emitThreadImage2, this, &VideoStreamer::catchFrame2);
    QObject::connect(worker2, &Worker::rectangleUpdated, this, &VideoStreamer::setRectangle);
    QObject::connect(worker2, &Worker::isdetect,this, &VideoStreamer::setdetect);
    QObject::connect(worker2, &Worker::rov_follow,this, &VideoStreamer::rov_follow2);
    tracker = TrackerCSRT::create();
    threadStreamer->start();
    threadStreamer2->start();
}
void VideoStreamer::streamerThreadSlot()
{

}

void VideoStreamer::grabFrame()
{
    if (!QThread::currentThread()->isInterruptionRequested()) {

        if(is_processing==false)
        {
            cap>>tempFrame;
            if(tempFrame.data)
            {
                //emit emitThreadImage(tempFrame);
            }
        }

    }
}

void VideoStreamer::start(uint32_t timeout)
{
    openVideoCamera("0");
    // Assume immediate success for initial compile testing, or implement proper thread signaling

}

void VideoStreamer::stop()
{
    if (!started()) {
        return;
    }

            //qCDebug(VideoStreamerLog) << "Stopping OpenCV Streamer thread...";

            // Use quit/wait to safely stop the worker thread
    threadStreamer->quit();
    if (!threadStreamer->wait(2000)) { // Wait up to 2 seconds
        //qCWarning(VideoStreamerLog) << "VideoStreamer thread failed to stop gracefully, terminating.";
        threadStreamer->terminate();
        threadStreamer->wait();
    }

    setStarted(false);
    emit onStopComplete(STATUS_OK);
}

void VideoStreamer::startDecoding(void* sink)
{
    //qCDebug(VideoStreamerLog) << "VideoStreamer::startDecoding (Dummy)";
    // For a custom receiver that handles its own rendering (e.g., QQuickFramebufferObject),
    // this method might be empty or used to signal the thread to begin rendering.
    // For now, it's a required implementation.
    emit onStartDecodingComplete(STATUS_OK);

}

void VideoStreamer::stopDecoding()
{
   //qCDebug(VideoStreamerLog) << "VideoStreamer::stopDecoding (Dummy)";
   // Required implementation.
}

void VideoStreamer::startRecording(const QString &fileName, FILE_FORMAT format)
{
   //qCDebug(VideoStreamerLog) << "VideoStreamer::startRecording (Dummy)";
   // Required implementation. You would hook up a video writer here.
}

void VideoStreamer::stopRecording()
{
   //qCDebug(VideoStreamerLog) << "VideoStreamer::stopRecording (Dummy)";
   // Required implementation.
}

void VideoStreamer::takeScreenshot(const QString &filename)
{
   //qCDebug(VideoStreamerLog) << "VideoStreamer::takeScreenshot (Dummy)";
   // Required implementation.
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

Worker::Worker(QObject *parent)
{
}

Worker::~Worker()
{

}

void Worker::grabImage()
{
    QMutexLocker locker(&mutex2);

    if(frame_to_be_processed.data)
    {
        Mat blob,frameCopy;
        Mat output;

        float x_factor=0.0,y_factor=0.0,x_center=0.0,y_center=0.0,w=0.0,h=0.0,conf=0.0;

        vector<Mat> outputs;
        // Apply Non-Maximum Suppression
        vector<int> indices;
        vector<Rect> boxes;
        vector<float> confidences;

        int rows=0;

        float box_conf=0.0;

        frame_to_be_processed.copyTo(frameCopy);

        is_processing=true;
        //is_box_detected=false;
        frameCount++;

        if (frameCount % 10 == 0 || !obj_tracking)               // No tracker yet
        {
            blobFromImage(frameCopy, blob, 1.0 / 255.0, Size(INPUT_WIDTH, INPUT_HEIGHT), Scalar(), true, false);
            net.setInput(blob);
            net.forward(outputs, net.getUnconnectedOutLayersNames());

            output = outputs[0];  // [1, 5, 8400]
            rows = output.size[2];

            x_factor = (float)frameCopy.size().width / INPUT_WIDTH;
            y_factor = (float)frameCopy.size().height / INPUT_HEIGHT;

            for (int i = 0; i < rows; ++i)
            {
                x_center = output.at<float>(0, 0, i);
                y_center = output.at<float>(0, 1, i);
                w = output.at<float>(0, 2, i);
                h = output.at<float>(0, 3, i);
                conf = output.at<float>(0, 4, i);

                if (conf >= CONFIDENCE_THRESHOLD)
                {

                    int left = int((x_center - 0.5f * w) * x_factor);
                    int top = int((y_center - 0.5f * h) * y_factor);
                    int width = int(w * x_factor);
                    int height = int(h * y_factor);

                    boxes.emplace_back(left, top, width, height);
                    confidences.push_back(conf);
                    //if (width > 0 && height > 0)

                       //dets.emplace_back(Rect2f(left, top, width, height), conf);

                }
            }

                    // Apply IOU-based tracking (keeps same IDs)
                    //iouTracker.update(dets, frame_idx++);

            dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, indices);

            if (!indices.empty())
            {

                // Pick highest confidence box manually (safe)
                int bestIdx = indices[0];
                float bestConf = confidences[bestIdx];
                float bestArea = boxes[indices[0]].area();

                for (size_t i = 1; i < indices.size(); ++i)
                {
                    int idx = indices[i];
                    float conf = confidences[idx];
                    double area = boxes[idx].area();

                            // 2️⃣ If confidence nearly equal, prefer bigger area
                    if (conf > bestConf || fabs(conf - bestConf) < 1e-3f)
                    {
                        if (area > bestArea)
                        {
                            bestConf = conf;
                            bestIdx = idx;
                            bestArea = area;
                        }

                        else if(fabs(area - bestArea) < 1.0f)
                        {
                            if (obj_tracking && !tracked_box.empty())
                            {
                                try
                                {
                                    cv::Point2d newCenter(
                                        boxes[idx].x + boxes[idx].width / 2.0,
                                        boxes[idx].y + boxes[idx].height / 2.0
                                        );

                                    cv::Point2d oldCenter(
                                        tracked_box.x + tracked_box.width / 2.0,
                                        tracked_box.y + tracked_box.height / 2.0
                                        );

                                    double distNew = cv::norm(newCenter - oldCenter);

                                    if (fabs(distNew) > 1e-3f)
                                        continue;// closer to currently tracked box

                                    else
                                    {
                                        bestIdx = idx;
                                        bestArea = area;
                                    }
                                }

                                catch (const cv::Exception& e)
                                {
                                    qDebug() << "⚠️ Proximity comparison failed:" << e.what();
                                }
                            }

                            else
                                continue;
                        }

                        else if(area < bestArea)
                            continue;
                    }

                    else if(conf< bestConf)
                    {
                        if (area > bestArea)
                        {

                            if (obj_tracking && !tracked_box.empty())
                            {
                                try
                                {
                                    cv::Point2d newCenter(
                                        boxes[idx].x + boxes[idx].width / 2.0,
                                        boxes[idx].y + boxes[idx].height / 2.0
                                        );
                                    cv::Point2d oldCenter(
                                        tracked_box.x + tracked_box.width / 2.0,
                                        tracked_box.y + tracked_box.height / 2.0
                                        );
                                    cv::Point2d bestCenter(
                                        boxes[bestIdx].x + boxes[bestIdx].width / 2.0,
                                        boxes[bestIdx].y + boxes[bestIdx].height / 2.0
                                        );

                                    double distNew = cv::norm(newCenter - oldCenter);
                                    double distBest = cv::norm(bestCenter - oldCenter);

                                    if (distNew < distBest)
                                    {
                                        bestIdx = idx; // closer to currently tracked box
                                        bestArea = area;
                                    }

                                    else
                                        continue;
                                }

                                catch (const cv::Exception& e)
                                {
                                    qDebug() << "⚠️ Proximity comparison failed:" << e.what();
                                }
                            }
                        }

                        else if(fabs(area - bestArea) < 1.0f || area < bestArea)
                            continue;
                    }
                }

                tracked_box = boxes[bestIdx];
                is_box_detected = true;

                        // Initialize tracker
                tracker->init(frameCopy, tracked_box);
                obj_tracking = true;
            }

            else
            {
                obj_tracking = false;
                is_box_detected=false;
            }


        }

        else if (obj_tracking)
        {
            // Update tracker
            if (!tracker->update(frameCopy, tracked_box))
            {
                // Tracking failed, force detection next frame
                obj_tracking = false;
                tracked_box = cv::Rect(); // clear stale box
                emit isdetect(false);
            }

            else
            {
                if(is_box_detected)
                {
                    emit rectangleUpdated(QRectF(tracked_box.x, tracked_box.y,tracked_box.width,tracked_box.height));
                    emit isdetect(true);

                    //int servo_signal = pll.update(tracked_box, frameCopy.size());

                    QPointF  frame_center(frameCopy.cols / 2.0, frameCopy.rows / 2.0);
                    QPointF  obj_center(tracked_box.x + tracked_box.width / 2.0,
                                       tracked_box.y + tracked_box.height / 2.0);
                    if(track_confirm)
                        emit rov_follow(frame_center,obj_center);
                    //rov.followDiver(frame_center, obj_center);

                    /*double offset_x = obj_center.x - frame_center.x; // horizontal
                    double offset_y = obj_center.y - frame_center.y; // vertical
                    double distance = sqrt(offset_x * offset_x + offset_y * offset_y);

                                      double max_x = frameCopy.cols / 2.0;
                                      double max_y = frameCopy.rows / 2.0;

                                        // Yaw: 0-359
                                        double yaw = (abs(offset_x) < DEADZONE) ? 180.0 : 180.0 + (offset_x / max_x) * 180.0;
                                        if (yaw < 0) yaw += 360;
                                        if (yaw >= 360) yaw -= 360;

                                          // Pitch: -90 to +90
                                          double pitch = (abs(offset_y) < DEADZONE) ? 0.0 : -(offset_y / max_y) * 90.0;

                                            // Roll: -180 to +180
                                            double roll = (abs(offset_x) < DEADZONE) ? 0.0 : (offset_x / max_x) * 180.0;

                                             // cout << "Yaw: " << yaw << "  Pitch: " << pitch << "  Roll: " << roll << endl;

                                                auto mapToServo = [](double offset, double maxOffset) {
                                                    double mapped = 1500 + (offset / maxOffset) * 500.0;
                                                    return std::clamp(mapped, 1000.0, 2000.0);
                                                };


                                                  double servo_x = mapToServo(offset_x, max_x); // Left/right
                                                  double servo_y = mapToServo(-offset_y, max_y); // Up/down (invert y)


                                                    Rect ref_rect(frame_center.x - (frameCopy.cols * ref_scale) / 2.0,
                                                                  frame_center.y - (frameCopy.rows * ref_scale) / 2.0,
                                                                  frameCopy.cols * ref_scale,
                                                                  frameCopy.rows * ref_scale);

                                                      double obj_area = tracked_box.area();
                                                      double ref_area = ref_rect.area();
                                                      double area_ratio = obj_area / ref_area;

                                                        QString moveX, moveY, moveZ;

                                                          // --- Horizontal (left-right) ---
                                                          if (abs(offset_x) > DEADZONE) {
                                                              moveX = (offset_x > 0) ? "RIGHT" : "LEFT";
                                                          } else moveX = "CENTER";

                                                            // --- Vertical (up-down) ---
                                                            if (abs(offset_y) > DEADZONE) {
                                                                moveY = (offset_y > 0) ? "DOWN" : "UP";
                                                            } else moveY = "CENTER";

                                                              // --- Depth (forward-backward) ---
                                                              if (area_ratio > 1.1)
                                                                  moveZ = "FORWARD";
                                                              else if (area_ratio < 0.9)
                                                                  moveZ = "BACKWARD";
                                                              else
                                                                  moveZ = "ALIGNED";

                                                                qDebug() << "MoveX:" << moveX
                                                                         << "MoveY:" << moveY
                                                                         << "MoveZ:" << moveZ
                                                                         << "AreaRatio:" << area_ratio
                                                                         << "Distance:" << distance;*/
                }
                else
                    emit isdetect(false);
            }
        }

        /*else
        {
            // Just predict new positions without YOLO (using Kalman)
                std::vector<std::pair<cv::Rect2f, float>> empty_dets;
            iouTracker.update(empty_dets, frame_idx++);
        }
        auto active_tracks = iouTracker.get_active_tracks();
        if (!active_tracks.empty())
        {
            // You can choose to emit *all* active track boxes
            for (const auto &track : active_tracks)
            {
                QRectF rect(track.bbox.x, track.bbox.y, track.bbox.width, track.bbox.height);
                emit rectangleUpdated(rect);
            }

                      // Optional: mark detection state
                      emit isdetect(true);
                      obj_tracking = true;
                  }
                  else
                  {
                      emit isdetect(false);
                      obj_tracking = false;
                  }*/

    }
    is_processing=false;
}

void Worker::grabFrame()
{
    if (!QThread::currentThread()->isInterruptionRequested()) {


            cap>>tempFrame;
            if(tempFrame.data)
            {
                //qDebug()<<"Frame";
                emit emitThreadImage(tempFrame);
            }

    }
}

QRectF Worker::rectangle() const
{
    //qDebug()<<m_rectangle.x()<<m_rectangle.width();
    return m_rectangle;

}

void VideoStreamer::setRectangle(const QRectF &rect)
{
    QMutexLocker locker(&mutex4);
    if (m_rectangle != rect) {
        m_rectangle = rect;
        //qDebug()<<m_rectangle.width();
        emit is_rectangle(m_rectangle);
    }
}

bool Worker::detect()
{
    return isdetected;
}

void VideoStreamer::setdetect(bool value)
{
    QMutexLocker locker(&mutex5);
    isdetected=value;
    //emit detectChanged();
    emit is_box(isdetected);
}

void VideoStreamer::rov_follow2(QPointF frame_center, QPointF obj_center)
{
    emit rov.followDiver(frame_center, obj_center);
}

bool Worker::nearlyEqual(float a, float b, float epsilon)
{
    return std::fabs(a - b) < epsilon;
}

void Worker::start(uint32_t timeout)
{
    //openVideoCamera("0");
    // Assume immediate success for initial compile testing, or implement proper thread signaling

}

void Worker::stop()
{
    if (!started()) {
        return;
    }

            //qCDebug(VideoStreamerLog) << "Stopping OpenCV Streamer thread...";

            // Use quit/wait to safely stop the worker thread
    threadStreamer->quit();
    if (!threadStreamer->wait(2000)) { // Wait up to 2 seconds
        //qCWarning(VideoStreamerLog) << "VideoStreamer thread failed to stop gracefully, terminating.";
        threadStreamer->terminate();
        threadStreamer->wait();
    }

    setStarted(false);
    emit onStopComplete(STATUS_OK);
}

void Worker::startDecoding(void* sink)
{
    //qCDebug(VideoStreamerLog) << "VideoStreamer::startDecoding (Dummy)";
    // For a custom receiver that handles its own rendering (e.g., QQuickFramebufferObject),
    // this method might be empty or used to signal the thread to begin rendering.
    // For now, it's a required implementation.
    emit onStartDecodingComplete(STATUS_OK);

}

void Worker::stopDecoding()
{
   //qCDebug(VideoStreamerLog) << "VideoStreamer::stopDecoding (Dummy)";
   // Required implementation.
}

void Worker::startRecording(const QString &fileName, FILE_FORMAT format)
{
   //qCDebug(VideoStreamerLog) << "VideoStreamer::startRecording (Dummy)";
   // Required implementation. You would hook up a video writer here.
}

void Worker::stopRecording()
{
   //qCDebug(VideoStreamerLog) << "VideoStreamer::stopRecording (Dummy)";
   // Required implementation.
}

void Worker::takeScreenshot(const QString &filename)
{
   //qCDebug(VideoStreamerLog) << "VideoStreamer::takeScreenshot (Dummy)";
   // Required implementation.
}
