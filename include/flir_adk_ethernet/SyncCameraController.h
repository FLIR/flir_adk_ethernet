/******************************************************************************/
/*                                                                            */
/*  Copyright (C) 2018, FLIR Systems                                          */
/*  All rights reserved.                                                      */
/*                                                                            */
/******************************************************************************/
#ifndef FLIR_ADK_ETHERNET_SYNCCAMERACONTROLLER_H
#define FLIR_ADK_ETHERNET_SYNCCAMERACONTROLLER_H

// C++ Includes
#include <string>

// ROS Includes
#include <ros/ros.h>
#include <std_msgs/Time.h>

#include "flir_adk_ethernet/BaseCameraController.h"
#include <flir_adk_ethernet/MultiTimeHeader.h>

using namespace std;

namespace flir_adk_ethernet
{
// class for controlling single camera synchronized with other camera(s)
class SyncCameraController : public BaseCameraController
{
  public:
    SyncCameraController();
    ~SyncCameraController();

  private:
    // virtual void onInit();
    virtual void setupExtraPubSub() override;
    virtual void setupFramePublish() override;
    void publishImage(const std_msgs::Time::ConstPtr& message);
    ros::Time timeFromNSec(uint64_t nsecs);

    ros::Publisher _timePublisher;
    ros::Subscriber _sub;
};

}

#endif
