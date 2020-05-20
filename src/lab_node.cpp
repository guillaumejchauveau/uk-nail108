#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>

void cb(const sensor_msgs::LaserScanConstPtr &msg) {
  ROS_INFO_STREAM("Range:" << *std::min_element(msg->ranges.begin(), msg->ranges.end()));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lab");
  ros::NodeHandle nodeHandle("~");

  std::string topic;
  if (!nodeHandle.getParam("topic", topic)) {
    ROS_ERROR("Could not find 'topic' parameter");
  }
  int queue_size;
  if (!nodeHandle.getParam("topic_queue_size", queue_size)) {
    ROS_ERROR("Could not find 'topic_queue_size' parameter");
  }
  //auto sub = nodeHandle.subscribe(topic, queue_size, &cb);

  ros::spin();
  return 0;
}
