/******************************************************************************/
/*                                                                            */
/*  Copyright (C) 2018, FLIR Systems                                          */
/*  All rights reserved.                                                      */
/*                                                                            */
/******************************************************************************/
// C++ Includes
#include <cstdlib>
#include <iostream>
#include <vector>
#include <numeric>

// ROS Includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <flir_adk_ethernet/MultiTimeHeader.h>


#define NUM_READINGS 60

using namespace std;

namespace flir_adk_ethernet
{
// nodelet for getting actual time difference between synced cameras
class TimeDifference : public nodelet::Nodelet {
  public:
    TimeDifference();
    ~TimeDifference();

  private:
    virtual void onInit();
    void calculateDifferences(const MultiTimeHeaderConstPtr& msg,
      MultiTimeHeader *header, MultiTimeHeader *otherHeader);

    ros::NodeHandle _nh, _pnh;
    ros::Subscriber _leftSub;
    ros::Subscriber _rightSub;

    MultiTimeHeader _leftHeader;
    MultiTimeHeader _rightHeader;

    std::vector<uint32_t> _timeDifferences;
};

}