// C++ Includes
#include <iostream>
#include <vector>
#include <numeric>

// ROS Includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>

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
    void calculateDifferences();
    void getImageHeader(const sensor_msgs::Image::ConstPtr& msg);
    void getActualTimeHeader(const std_msgs::Header::ConstPtr& msg);
    void addTimeDiff();

    ros::NodeHandle _nh, _pnh;
    ros::Subscriber _imgSubscriber;
    ros::Subscriber _actualTimeSubscriber;
    std_msgs::Header _imgHeader;
    std_msgs::Header _actualTimeHeader;

    std::vector<uint32_t> _timeDifferences;
};

}