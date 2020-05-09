#include <ros/ros.h>
#include "lab/RosPackageTemplate.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_package_template");
  ros::NodeHandle nodeHandle("~");

  lab::RosPackageTemplate rosPackageTemplate(nodeHandle);
  //husky_highlevel_controller::HuskyHighlevelController huskyHighlevelController(nodeHandle);

  ros::spin();
  return 0;
}
