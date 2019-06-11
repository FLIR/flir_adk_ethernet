#ifndef FLIR_BOSON_ETHERNET_IMAGEEVENTHANDLER_H
#define FLIR_BOSON_ETHERNET_IMAGEEVENTHANDLER_H

// C++ Includes
#include <string>
#include <mutex>

// ROS Includes
#include <ros/ros.h>

// Spinnaker Includes
#include <spinnaker/Spinnaker.h>
#include <spinnaker/SpinGenApi/SpinnakerGenApi.h>

using namespace std;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

struct ImageInfo {
  int32_t width, height, size;
};

class ImageEventHandler : public ImageEvent
{
public:
    // The constructor retrieves the serial number and initializes the image 
    // counter to 0.
    ImageEventHandler(CameraPtr pCam);
    ~ImageEventHandler();

    ImageInfo GetImageInfo();
    void Init(uint8_t *buffer);

    // This method defines an image event. In it, the image that triggered the 
    // event is converted and saved before incrementing the count. Please see 
    // Acquisition_CSharp example for more in-depth comments on the acquisition 
    // of images.
    void OnImageEvent(ImagePtr image) override;
    bool IsValid();


private:
    string m_deviceSerialNumber;
    std::mutex m_imageWriteMutex;
    uint8_t *m_bufferStart;
    ImageInfo m_imageInfo;
    bool m_isValid;
};

#endif