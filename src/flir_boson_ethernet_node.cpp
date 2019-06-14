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

  nodelet::M_string leftRemap(ros::names::getRemappings());
  nodelet::M_string rightRemap;

  leftRemap.insert({"image_raw", "left/image_raw"});
  rightRemap.insert({"image_raw", "right/image_raw"});

  leftRemap.insert({"ip_addr", "left_ip"});
  rightRemap.insert({"ip_addr", "right_ip"});

  for(auto it = leftRemap.begin(); it != leftRemap.end(); ++it) {
    std::cout << it->first.c_str() << " " << it->second.c_str() << std::endl;
  }

  for(auto it = remap.begin(); it != remap.end(); ++it) {
    std::cout << it->first.c_str() << " " << it->second.c_str() << std::endl;
  }
  ros::NodeHandle nh("~");
  std::string leftIP;
  std::string rightIP;
  nh.param<std::string>("left_ip", leftIP, "");
  nh.param<std::string>("right_ip", rightIP, "");

  std::string nodelet_name = ros::this_node::getName();
  if(!leftIP.empty()) {
    std::cout << "LEFT NODE " << leftIP << std::endl;
    nodelet.load(nodelet_name + "_left", 
      "flir_boson_ethernet/CameraController", leftRemap, nargv);
  }
  if(!rightIP.empty()) {
    nodelet.load(nodelet_name + "_right", 
      "flir_boson_ethernet/CameraController", rightRemap, nargv);
  }
  if(leftIP.empty() && rightIP.empty()) {
    nodelet.load(nodelet_name, "flir_boson_ethernet/CameraController", remap, nargv);
  }
  ros::spin();
  return 0;
}