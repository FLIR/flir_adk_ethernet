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
#include <flir_boson_ethernet/MultiTimeHeader.h>


#define NUM_READINGS 60

using namespace std;

namespace flir_boson_ethernet
{

class TimeDifference : public nodelet::Nodelet {
  public:
    TimeDifference();
    ~TimeDifference();

  private:
    virtual void onInit();
    void calculateDifferences(const MultiTimeHeaderConstPtr& leftMsg,
      const MultiTimeHeaderConstPtr& rightMsg);
    // void getImageHeader(const sensor_msgs::Image::ConstPtr& msg);
    // void getActualTimeHeader(const std_msgs::Header::ConstPtr& msg);
    // void addTimeDiff();

    ros::NodeHandle _nh, _pnh;
    // MultiTimeHeader _leftHeader;
    // MultiTimeHeader _rightHeader;

    // message_filters::TimeSynchronizer<MultiTimeHeader, MultiTimeHeader> _sync;

    std::vector<uint32_t> _timeDifferences;
};

}