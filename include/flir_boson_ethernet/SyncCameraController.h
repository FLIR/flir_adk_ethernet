#ifndef FLIR_BOSON_ETHERNET_SYNCCAMERACONTROLLER_H
#define FLIR_BOSON_ETHERNET_SYNCCAMERACONTROLLER_H

// C++ Includes
#include <string>

// ROS Includes
#include <ros/ros.h>

#include "flir_boson_ethernet/BaseCameraController.h"
#include <flir_boson_ethernet/MultiTimeHeader.h>

using namespace std;

namespace flir_boson_ethernet
{

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
