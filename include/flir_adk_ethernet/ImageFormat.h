/******************************************************************************/
/*                                                                            */
/*  Copyright (C) 2018, FLIR Systems                                          */
/*  All rights reserved.                                                      */
/*                                                                            */
/******************************************************************************/
#ifndef FLIR_ADK_ETHERNET_IMAGEFORMAT_H
#define FLIR_ADK_ETHERNET_IMAGEFORMAT_H

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
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

namespace flir_adk_ethernet
{
// class for managing properties assosicated with pixel formats 
// (color, mono, 8 and 16 bit)
class ImageFormat {
  public:
    ImageFormat(std::string format);
    ImageFormat(const ImageFormat& obj);
    ~ImageFormat();

    // gets the integer value for setting the node
    int getValue(CEnumerationPtr nodePtr);
    // gets the node name for setting the node
    gcstring getNodeName();
    // gets bytes per pixel for sizing the image buffer
    int getBytesPerPixel();
    // returns the string to present to the user
    std::string toString();
    // gets the OpenCV integer value
    int getMatType();
    // gets the ROS image encoding 
    std::string getImageEncoding();
    // returns the Spinnaker format enum
    PixelFormatEnums getFormat();

  private:
    PixelFormatEnums _format;
};

}

#endif