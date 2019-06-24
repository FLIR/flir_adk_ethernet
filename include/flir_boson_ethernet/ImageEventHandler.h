#ifndef FLIR_BOSON_ETHERNET_IMAGEEVENTHANDLER_H
#define FLIR_BOSON_ETHERNET_IMAGEEVENTHANDLER_H

// C++ Includes
#include <string>
#include <mutex>

#include <chrono>

// ROS Includes
#include <ros/ros.h>

// Spinnaker Includes
#include <spinnaker/Spinnaker.h>
#include <spinnaker/SpinGenApi/SpinnakerGenApi.h>

#include "../spinnaker_wrappers/CameraWrapper.h"

using namespace std;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

typedef std::chrono::high_resolution_clock Clock;

namespace flir_boson_ethernet {

struct ImageInfo {
  int32_t width, height, size;
};

class ImageEventHandler : public ImageEvent {
  public:
    // The constructor retrieves the serial number and initializes the image 
    // counter to 0.
    ImageEventHandler(std::shared_ptr<CameraWrapper> pCam);
    ImageEventHandler(const ImageEventHandler& handler);
    ~ImageEventHandler();

    void setGrayscale(bool gray);

    void Init();

    // This method defines an image event. In it, the image that triggered the 
    // event is converted and saved before incrementing the count. Please see 
    // Acquisition_CSharp example for more in-depth comments on the acquisition 
    // of images.
    void OnImageEvent(ImagePtr image) override;
    void *GetImageData();
    ImageInfo GetImageInfo();
    uint64_t GetCaptureTime();

  private:
    std::shared_ptr<CameraWrapper> _pCam;
    string m_deviceSerialNumber;
    ImagePtr m_resultImage;
    // uint8_t *m_imageBuffer;
    std::mutex m_mutex;
    bool m_grayscale = false;

    std::chrono::_V2::system_clock::time_point startTime;
    uint64_t m_lastTimeStamp;
};

}

#endif