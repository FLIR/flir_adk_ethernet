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

#include "flir_boson_ethernet/BaseCameraController.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

namespace flir_boson_ethernet
{

class CameraController : public BaseCameraController
{
  public:
    CameraController();
    ~CameraController();

  private:
    virtual void setupFramePublish() override;
    void captureAndPublish(const ros::TimerEvent& evt);

    ros::Timer capture_timer;

    // Default Program options
    float _frameRate;
    bool zoom_enable;
    SensorTypes sensor_type;
    Encoding _videoMode;
};

}  // namespace flir_boson_ethernet

#endif  // FLIR_BOSON_ETHERNET_BOSONCAMERA_H
