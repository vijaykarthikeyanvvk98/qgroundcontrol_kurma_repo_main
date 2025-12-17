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
#include <opencv2/core.hpp> // Basic OpenCV structures (cv::Mat)
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp> // Video write#include <iostream>
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

static VideoWriter video;

static int frameCount = 0;
static int _video_format;
static bool track_confirm=false;
static bool recording_status = false;

// --------------------------------
cv::Mat frame,newFrame,frame_to_be_processed;
QTimer tUpdate;
QString streaming_path = "";
int calculated_difference=0;
QTimer *timer=nullptr,*timer2=nullptr;
cv::Mat tempFrame;


static cv::VideoCapture cap;
//

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

int frame_width=0,frame_height=0;
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
QString copyONNXToWritablePath() {
    QString assetPath = ":/models/best.onnx"; // resource path
    QString destDir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    QDir().mkpath(destDir);
    QString destFile = destDir + "/best.onnx";

    QFile file(assetPath);
    if (!file.exists()) return QString();

    file.copy(destFile); // copy resource to real file
    return destFile;
}

VideoStreamer::VideoStreamer(QObject *parent)
{
    // set target for ROV
    //rov.setTarget("192.168.2.2", 14550);
    //rov.start();
    // Initialize with a default rectangle from C++
    threadStreamer  = new QThread();
    threadStreamer2 = new QThread();

    connect(&tUpdate, &QTimer::timeout, this, &VideoStreamer::streamVideo);
    timer = new QTimer(this);
    timer2 = new QTimer(this);
    QString modelPath = copyONNXToWritablePath();
    net = cv::dnn::readNetFromONNX(modelPath.toStdString());
    //net = readNetFromONNX(MODEL_PATH);
    net.setPreferableBackend(DNN_BACKEND_OPENCV);
    net.setPreferableTarget(DNN_TARGET_CPU);
}

VideoStreamer::~VideoStreamer()
{

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

    if (recording_status)
    {
        video.write(frame);
    }
}




void VideoStreamer::openVideoCamera(QString path)
{

    if(cap.isOpened())
        cap.release();
    else;

    cap.open("udpsrc port=5600 ! "
        "application/"
        "x-rtp,encoding-name=H264,payload="
        "96 ! rtph264depay   ! avdec_h264  ! videoconvert ! appsink drop=true sync=false max-buffers=1", cv::CAP_GSTREAMER);


    if(!cap.isOpened())
        qDebug()<<"Error";

            // 👈 Owned by this
    Worker *worker1= new Worker();
    Worker *worker2= new Worker();
    worker1->moveToThread(threadStreamer);
    worker2->moveToThread(threadStreamer2);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);


    if (cap.get(cv::CAP_PROP_FPS) <= 0)
    {
        timer->setInterval(1000/40);
        tUpdate.start(1000 / 10);
        timer2->setInterval(1000/40);

        fps = 25;
        calculated_difference = 1000 / 25;

    }
    else
    {
        timer->setInterval(1000/cap.get(cv::CAP_PROP_FPS));
        timer2->setInterval((1000/cap.get(cv::CAP_PROP_FPS)));
        tUpdate.start(1000 /cap.get(cv::CAP_PROP_FPS));
        fps=cap.get(cv::CAP_PROP_FPS);
        calculated_difference = 1000 / cap.get(cv::CAP_PROP_FPS);

    }
    connect(timer, &QTimer::timeout, worker1, &Worker::grabFrame);
    connect(timer2, &QTimer::timeout, worker2, &Worker::grabImage);

    QObject::connect(threadStreamer, SIGNAL(started()), timer, SLOT(start()));
    QObject::connect(threadStreamer2, SIGNAL(started()), timer2, SLOT(start()));
    QObject::connect(worker1, &Worker::emitThreadImage, this, &VideoStreamer::catchFrame);
    QObject::connect(worker2, &Worker::rectangleUpdated, this, &VideoStreamer::setRectangle);
    QObject::connect(worker2, &Worker::isdetect,this, &VideoStreamer::setdetect);
    QObject::connect(worker2, &Worker::rov_follow,this, &VideoStreamer::rov_follow2);
    //tracker = TrackerCSRT::create();
    threadStreamer->start();
    threadStreamer2->start();
    setStarted(true);
    emit onStartComplete(STATUS_OK);

}




void VideoStreamer::start(uint32_t timeout)
{
    /*if (cap.isOpened()) {
        emit onStartComplete(STATUS_INVALID_STATE);
        return;
    }*/

    /*if (_uri.isEmpty()) {
        emit onStartComplete(STATUS_INVALID_URL);
        return;
    }*/

            //openVideoCamera(_uri);
    openVideoCamera(VIDEO_PATH.data());
    //openVideoCamera("0");
    // Assume immediate success for initial compile testing, or implement proper thread signaling

}

void VideoStreamer::stop()
{
    recording_status=false;
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

    if (!cap.isOpened()) {
        emit onStartDecodingComplete(STATUS_INVALID_STATE);
        return;
    }

    if(cap.isOpened())
    {
        if(video.isOpened())
            video.release();
        cap.release();
    }
    emit onStopDecodingComplete(STATUS_OK);
}

void VideoStreamer::startRecording(const QString &fileName, FILE_FORMAT format)
{
   //qCDebug(VideoStreamerLog) << "VideoStreamer::startRecording (Dummy)";
   // Required implementation. You would hook up a video writer here.
    //recording_status = true;
    if(!cap.isOpened())
    {
        emit onStartRecordingComplete(STATUS_FAIL);
        return;
    }
    if(frame_width == 0)
    {
        frame_width= cap.get(cv::CAP_PROP_FRAME_WIDTH);
        frame_height= cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    }
    switch (format) {
        case FILE_FORMAT_MKV:
            _video_format = cv::VideoWriter::fourcc('X', '2', '6', '4');
            break;
        case FILE_FORMAT_MOV:
            _video_format = cv::VideoWriter::fourcc('A', 'V', 'C', '1');
            break;
        case FILE_FORMAT_MP4:
            _video_format = cv::VideoWriter::fourcc('M', 'P', '4', 'V');
            break;
        default:
            // QMediaFormat::AVI, WMV, Ogg, WebM
            //_video_format =cv::VideoWriter::fourcc('A', 'V', 'C', '1');
            //cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
            _video_format = cv::VideoWriter::fourcc('M', 'P', '4', 'V');
            break;
    }
    video.open(fileName.toStdString(),
               _video_format,
               fps,
               Size(frame_width,frame_height),
               true);

    if (!video.isOpened()) {
        qDebug() << "Error: Could not open the video writer.";
        recording_status = false;
    }
    else
    {
        recording_status = true;

    }

    emit onStartRecordingComplete(STATUS_OK);

}

void VideoStreamer::stopRecording()
{
   //qCDebug(VideoStreamerLog) << "VideoStreamer::stopRecording (Dummy)";
   // Required implementation.
    recording_status = false;

    if (video.isOpened()) {
        video.release();
        //export_video();

    }

    else;

    emit onStopRecordingComplete(STATUS_OK);
}

void VideoStreamer::takeScreenshot(const QString &filename)
{
   //qCDebug(VideoStreamerLog) << "VideoStreamer::takeScreenshot (Dummy)";
   // Required implementation.
    // Acquire mutex to safely access the shared global 'frame' Mat
    QMutexLocker locker(&g_frameAccessMutex);

    if (frame.empty()) {
        qDebug() << "Cannot take screenshot: Frame is empty.";
        return;
    }
    Mat frameCopy;

    frame.copyTo(frameCopy);

            // 1. Determine the save directory: "Pictures/VideoScreenshots"
            // This is OS-standard compliant for user-saved media.
    /*QString saveDir = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation) + "/VideoScreenshots";
    QDir dir(saveDir);

     // Create the directory if it doesn't exist
     if (!dir.exists()) {
         if (!dir.mkpath(".")) {
             qDebug() << "Failed to create save directory:" << saveDir;
             return;
         }
     }

              // 2. Generate a unique, timestamped filename (e.g., Screenshot_20251113_174400.png)
      QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
      QString fullPath = saveDir + QString("/Screenshot_%1.png").arg(timestamp);*/

            // 3. Save the frame using OpenCV (PNG is lossless and generally preferred)
    try {
        // frame is a global cv::Mat, we save it directly.
        //bool success = cv::imwrite(fullPath.toStdString(), frameCopy);
        bool success = cv::imwrite(filename.toStdString(), frameCopy);

        if (success) {
            //qDebug() << "✅ Screenshot saved successfully to:" << fullPath;
            qDebug() << "✅ Screenshot saved successfully to:" << filename;

        } else {
            //qDebug() << "❌ Failed to save screenshot at:" << fullPath;
            qDebug() << "❌ Failed to save screenshot at:" << filename;
            emit onTakeScreenshotComplete(STATUS_NOT_IMPLEMENTED);

        }
    } catch (const cv::Exception& e) {
        qDebug() << "❌ OpenCV Error during imwrite:" << e.what();
    }
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

        vector<pair<cv::Rect2f, float>> dets;  // (bbox, conf)

        int rows=0;

        float box_conf=0.0;

        frame_to_be_processed.copyTo(frameCopy);

        is_processing=true;

        frameCount++;

        if (frameCount % 5 == 0 || !obj_tracking)               // No tracker yet
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

                            //boxes.emplace_back(left, top, width, height);
                            //confidences.push_back(conf);
                            //if (width > 0 && height > 0)
                    dets.emplace_back(Rect2f(left,top,width,height), conf);

                }
            }

                    // Apply IOU-based tracking (keeps same IDs)
            iouTracker.update(dets, frame_idx++);

        }

        else
        {
            // Just predict new positions without YOLO (using Kalman)
            std::vector<std::pair<cv::Rect2f, float>> empty_dets;
            iouTracker.update(empty_dets, frame_idx++);

                    //emit isdetect(false);
                    //obj_tracking = false;
        }

                // ---- 3️⃣ Emit updated rectangles ----
        auto active_tracks = iouTracker.get_active_tracks();

        if (!active_tracks.empty())
        {
            for (const auto &track : active_tracks)
            {
                QRectF rect(track.bbox.x, track.bbox.y, track.bbox.width, track.bbox.height);
                emit rectangleUpdated(rect);

                QPointF  frame_center(frameCopy.cols / 2.0, frameCopy.rows / 2.0);
                QPointF  obj_center(track.bbox.x + track.bbox.width / 2.0,
                                   track.bbox.y + track.bbox.height / 2.0);
                if(track_confirm)
                    emit rov_follow(frame_center,obj_center);
            }

            emit isdetect(true);
            obj_tracking = true;
        }
        else
        {
            emit isdetect(false);
            obj_tracking = false;
        }

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


void VideoStreamer::setRectangle(const QRectF &rect)
{
    QMutexLocker locker(&mutex4);
    if (m_rectangle != rect) {
        m_rectangle = rect;
        //qDebug()<<m_rectangle.width();
        emit is_rectangle(m_rectangle);
    }
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
    rov.followDiver(frame_center, obj_center);
}

bool Worker::nearlyEqual(float a, float b, float epsilon)
{
    return std::fabs(a - b) < epsilon;
}
