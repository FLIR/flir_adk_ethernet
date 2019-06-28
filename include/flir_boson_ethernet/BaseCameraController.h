#ifndef FLIR_BOSON_ETHERNET_BASECAMERACONTROLLER_H
#define FLIR_BOSON_ETHERNET_BASECAMERACONTROLLER_H

// C++ Includes
#include <string>

// Linux system includes
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/videodev2.h>

// OpenCV Includes
#include <opencv2/opencv.hpp>

// ROS Includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

// messages
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "flir_boson_ethernet/SharedTypes.h"
#include "flir_boson_ethernet/EthernetCamera.h"
#include <flir_boson_ethernet/MultiTimeHeader.h>

using namespace std;

namespace flir_boson_ethernet
{

// abstract class nodelet for controlling an ethernet camera
class BaseCameraController : public nodelet::Nodelet
{
  public:
    BaseCameraController();
    virtual ~BaseCameraController();

  protected:
    // required for nodelet - handles initialization
    virtual void onInit();
    // sets up publishers and subscribers beyond the minimal ones
    virtual void setupExtraPubSub();
    // abstract method - handles how to trigger frames get published to topic
    virtual void setupFramePublish() = 0;
    // sets up subscribers for command and control
    virtual void setupCommandListeners();
    // publishes frame to topic
    void publishImage(ros::Time timestamp);
    

    // command listeners
    void setPixelFormat(const std_msgs::StringConstPtr& msg);
    void setAutoFFC(const std_msgs::BoolConstPtr& msg);
    void executeFFC();

    ros::NodeHandle nh, pnh;
    uint64_t _seq = 0;
    std::shared_ptr<image_transport::ImageTransport> it;
    cv_bridge::CvImage _cvImage;
    image_transport::CameraPublisher _imagePublisher;
    EthernetCamera *_camera;
    std::string frame_id, video_mode_str;

    ros::Subscriber _pixelFormatListener;
    ros::Subscriber _autoFFCListener;
    ros::Subscriber _ffcListener;
};

}

#endif
