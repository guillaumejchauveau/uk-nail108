#include <ros/ros.h>
#include "lab/CrashDetector.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "crash");
  ros::NodeHandle nodeHandle("~");

  lab::CrashDetector detector(nodeHandle);

  ros::spin();
  return 0;
}
