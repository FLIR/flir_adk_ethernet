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

// Spinnaker Includes
#include <spinnaker/Spinnaker.h>
#include <spinnaker/SpinGenApi/SpinnakerGenApi.h>

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
    void agcBasicLinear(const cv::Mat& input_16,
                        cv::Mat* output_8,
                        const int& height,
                        const int& width);
    void captureAndPublish(const ros::TimerEvent& evt);

    ros::NodeHandle nh, pnh;
    std::shared_ptr<image_transport::ImageTransport> it;
    image_transport::CameraPublisher image_pub;
    sensor_msgs::ImagePtr pub_image;
    ros::Timer capture_timer;
    std::vector<EthernetCamera> _cameras;

    // Default Program options
    std::string frame_id, video_mode_str;
    float _frameRate;
    Encoding video_mode;
    bool zoom_enable;
    SensorTypes sensor_type;
};

}  // namespace flir_boson_ethernet

#endif  // FLIR_BOSON_ETHERNET_BOSONCAMERA_H
