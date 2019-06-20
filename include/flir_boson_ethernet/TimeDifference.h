// ROS Includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Header.h>

using namespace std;

namespace flir_boson_ethernet
{

class TimeDifference : public nodelet::Nodelet {
  public:
    TimeDifference();
    ~TimeDifference();

  private:
    virtual void onInit();
    void calculateDifferences(const std_msgs::Header::ConstPtr& msg);

    ros::NodeHandle _nh, _pnh;
    ros::Subscriber _subscriber;
};

}