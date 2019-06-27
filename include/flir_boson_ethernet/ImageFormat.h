#ifndef FLIR_BOSON_ETHERNET_IMAGEFORMAT_H
#define FLIR_BOSON_ETHERNET_IMAGEFORMAT_H

// C++ Includes
#include <iostream>
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

// Spinnaker Includes
#include <spinnaker/Spinnaker.h>
#include <spinnaker/SpinGenApi/SpinnakerGenApi.h>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

namespace flir_boson_ethernet
{

class ImageFormat {
  public:
    ImageFormat(std::string format);
    ImageFormat(const ImageFormat& obj);
    ~ImageFormat();

    int getValue(CEnumerationPtr nodePtr);
    gcstring getNodeName();
    int getBytesPerPixel();
    std::string toString();
    int getMatType();
    std::string getImageEncoding();
    PixelFormatEnums getFormat();

  private:
    PixelFormatEnums _format;
};

}

#endif