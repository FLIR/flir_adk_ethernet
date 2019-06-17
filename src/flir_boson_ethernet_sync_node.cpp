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
  leftRemap.insert({"ip_addr", "left_ip"});

  nodelet::M_string rightRemap(ros::names::getRemappings());
  rightRemap.insert({"ip_addr", "right_ip"});
  
  nodelet.load(nodelet_name, "flir_boson_ethernet/SyncCameraController", 
    leftRemap, nargv);
//   nodelet.load(nodelet_name, "flir_boson_ethernet/SyncCameraController", 
//     rightRemap, nargv);
  ros::spin();
  return 0;
}