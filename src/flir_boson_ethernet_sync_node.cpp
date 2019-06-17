#include <string>

// ROS Includes
#include <ros/ros.h>
#include <nodelet/loader.h>
#include <ros/console.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Boson Camera Node");
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();

  nodelet::M_string leftRemap(ros::names::getRemappings());
  leftRemap.insert({"left_ip", "ip_addr"});

  nodelet::M_string rightRemap(ros::names::getRemappings());
  rightRemap.insert({"right_ip", "ip_addr"});
  
  nodelet.load(nodelet_name + "_left", "flir_boson_ethernet/SyncCameraController", 
    leftRemap, nargv);
  nodelet.load(nodelet_name + "_right", "flir_boson_ethernet/SyncCameraController", 
    rightRemap, nargv);
  ros::spin();
  return 0;
}