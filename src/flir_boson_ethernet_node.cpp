#include <string>

// ROS Includes
#include <ros/ros.h>
#include <nodelet/loader.h>
#include <ros/console.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Boson Camera Node");
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  // for(ros::M_string::const_iterator it = remap.cbegin(); it != remap.cend(); ++it)
  // {
  //     ROS_INFO("%s %s", it->first.c_str(), it->second.c_str());
  // }
  nodelet.load(nodelet_name, "flir_boson_ethernet/BosonCamera", remap, nargv);
  nodelet.load("image_view", "image_view/image", remap, nargv);
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  return 0;
}