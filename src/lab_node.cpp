#include <ros/ros.h>
#include <lab/LaserTracker.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "lab");
  ros::NodeHandle nodeHandle("~");

  lab::LaserTracker tracker(nodeHandle);

  ros::spin();
  return 0;
}
