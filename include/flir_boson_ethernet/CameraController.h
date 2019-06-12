#ifndef FLIR_BOSON_ETHERNET_CAMERACONTROLLER_H
#define FLIR_BOSON_ETHERNET_CAMERACONTROLLER_H

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

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "flir_boson_ethernet/EthernetCamera.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

namespace flir_boson_ethernet
{

enum Encoding
{
  YUV = 0,
  RAW16 = 1
};

enum SensorTypes
{
  Boson320,
  Boson640
};

class CameraController : public nodelet::Nodelet
{
  public:
    CameraController();
    ~CameraController();

  private:
    virtual void onInit();
    void captureAndPublish(const ros::TimerEvent& evt);

    ros::NodeHandle nh, pnh;
    std::shared_ptr<image_transport::ImageTransport> it;
    cv_bridge::CvImage _cvImage;
    image_transport::CameraPublisher _imagePublisher;
    sensor_msgs::ImagePtr _publishedImage;
    ros::Timer capture_timer;
    EthernetCamera *_camera;

    // Default Program options
    std::string frame_id, video_mode_str;
    float _frameRate;
    Encoding video_mode;
    bool zoom_enable;
    SensorTypes sensor_type;
    Encoding _videoMode;
};

}  // namespace flir_boson_ethernet

#endif  // FLIR_BOSON_ETHERNET_BOSONCAMERA_H
