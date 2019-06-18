#include <string>

// ROS Includes
#include <ros/ros.h>
#include <nodelet/loader.h>
#include <ros/console.h>

int main(int argc, char** argv)
{
  // ros::init(argc, argv, "Boson Camera Node");
  // nodelet::Loader nodelet;
  // nodelet::M_string remap(ros::names::getRemappings());
  // nodelet::V_string nargv;
  // std::string nodelet_name = ros::this_node::getName();

  // nodelet.load(nodelet_name, "flir_boson_ethernet/CameraController", remap, nargv);
  ros::spin();
  return 0;
}