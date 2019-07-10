#ifndef FLIR_BOSON_ETHERNET_ETHERNETCAMERA_H
#define FLIR_BOSON_ETHERNET_ETHERNETCAMERA_H

// C++ Includes
#include <iostream>
#include <string>
#include <locale>

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
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

// Spinnaker Includes
#include <spinnaker/Spinnaker.h>
#include <spinnaker/SpinGenApi/SpinnakerGenApi.h>

// Local Includes
#include "flir_boson_ethernet/ImageEventHandler.h"
#include "flir_boson_ethernet/ImageFormat.h"
#include "../spinnaker_wrappers/SystemWrapper.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

namespace flir_boson_ethernet
{

// configuration parameters for an ethernet camera
struct EthernetCameraInfo {
    string ip, camInfoPath, pixelFormat, camType;
    int width = 0;
    int height = 0;
    int xOffset = 0;
    int yOffset = 0;
};

enum Polarity {

};

// Class for interacting with any type of ethernet camera (Boson or Blackfly)
// have been tested so far
class EthernetCamera
{
  public:
    EthernetCamera(EthernetCameraInfo info, 
      std::shared_ptr<SystemWrapper> sys, ros::NodeHandle);
    ~EthernetCamera();

    // AGC Sample ONE: Linear from min to max.
    // Input is a MATRIX (height x width) of 16bits. (OpenCV mat)
    // Output is a MATRIX (height x width) of 8 bits (OpenCV mat)
    // unimplemented
    void agcBasicLinear(const cv::Mat& input_16,
                    cv::Mat* output_8,
                    const int& height,
                    const int& width);
    bool openCamera();
    bool closeCamera();

    // gets the openCV image matrix
    cv::Mat getImageMatrix();
    
    // gets the camera info formatted as a ROS sensor message
    sensor_msgs::CameraInfo getCameraInfo();

    // gets the camera-reported frame timestamp (distinct from the ROS reported one)
    uint64_t getActualTimestamp();

    // sets the pixel format allowed values: mono_8, mono_16, color_8, color_16
    std::string setPixelFormat(std::string format);

    // activates FFC shutter (only works with Boson)
    std::string performFFC();

    // sets auto FFC to Auto (true) or Manual (false) (only works with Boson)
    std::string setAutoFFC(bool autoFFC);
    void setPolarity(Polarity pol);

    std::string getNodeValue(std::string nodeName);
    bool setNodeValue(std::string nodeName, std::string value);

    // gets encoding for image conversion
    std::string getEncoding();
    
    // sets ROI of camera view
    void setROI(int xOffset, int yOffset, int width, int height);
    void setCenterROI(int width, int height);

  private:
    PixelFormatEnums getPixelFormat(string formatStr);

    // open camera helpers
    bool findMatchingCamera(CameraListWrapper camList, const unsigned int numCams);
    void initPixelFormat();
    bool setImageInfo();
    void setCameraEvents();
    bool setImageAcquisition();
    void initOpenCVBuffers();
    void setCameraInfo();

    // cleans up camera events
    void unsetCameraEvents();

    // creates image buffer that backs the openCV matrix
    void createBuffer();

    int getPixelSize();
    bool ipMatches(string ip, INodeMap& nodeMapTLDevice);
    bool camTypeMatches(string camType, INodeMap& nodeMapTLDevice);

    // gets the pixel format as a string e.g. color_8, mono_16
    std::string formatToString(PixelFormatEnums format);

    void stopCapture();
    void startCapture();

    bool setStringNode(CNodePtr node, std::string value);
    bool setIntNode(CNodePtr node, int value);
    bool setFloatNode(CNodePtr node, float value);
    bool setBoolNode(CNodePtr node, bool value);
    bool setEnumNode(CNodePtr node, std::string value);
    bool setCommandNode(CNodePtr node);

    std::shared_ptr<camera_info_manager::CameraInfoManager> _cameraInfo;
    int32_t _width, _height, _xOffset, _yOffset, _imageSize;
    int32_t _frame = 0;                // First frame number enumeration
    uint8_t *_bufferStart;
    std::shared_ptr<CameraWrapper> _pCam;
    std::shared_ptr<SystemWrapper> _system;
    std::shared_ptr<ImageEventHandler> _imageHandler;

    cv::Mat _thermalImageMat;

    // Default Program options
    std::string _ipAddr, _cameraInfoPath;
    bool _zoomEnable;
    ImageFormat _selectedFormat;
    std::string _camType;
};

}  // namespace flir_boson_ethernet

#endif