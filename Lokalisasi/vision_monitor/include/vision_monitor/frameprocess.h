#ifndef FRAMEPROCESS_H
#define FRAMEPROCESS_H

#include <QtCore>
#include <QtGui>
#include <QtWidgets>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>

class FrameProcess:public QThread{
    Q_OBJECT
public:
    FrameProcess(QObject *parent = 0);
    ~FrameProcess();

    void play();
    void stop();

protected:
    void run();

private:
    QMutex mutex_;
    QWaitCondition wait_cond_;

    ros::NodeHandle nh_;

    sensor_msgs::ImagePtr img_ptr_;

    cv_bridge::CvImagePtr cv_img_ptr_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber it_subs_;

    void imgCallback(const sensor_msgs::ImageConstPtr &_msg);

    int encoding_;

    cv::Mat frame_;

    int frame_rate_;

    bool running;

signals:
    void capturedFrame(const QImage &_qimage);
};

#endif // FRAMEPROCESS_H
