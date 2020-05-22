#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

namespace lab {

class LaserTracker {
protected:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber laserSubscriber_;
  ros::Publisher cmdVelPublisher_;
  ros::Publisher vizPublisher_;

  void laserScanCallback(const sensor_msgs::LaserScanConstPtr &msg) {
    size_t target_start = 0;
    size_t target_stop = 0;
    bool target_found = false;
    size_t i = 0;
    float target_min_range;
    for (const auto &range : msg->ranges) {
      if (range < msg->range_max) {
        if (!target_found) {
          target_start = i;
          target_found = true;
          target_min_range = range;
        } else if (range < target_min_range) {
          target_min_range = range;
        }
      } else if (target_found) {
        target_stop = i - 1;
        break;
      }
      i++;
    }

    if (!target_found) {
      return;
    }

    auto target_center_angle =
      (target_stop + target_start) / 2 * msg->angle_increment + msg->angle_min;
    auto target_center_angle_abs = fabs(target_center_angle);

    geometry_msgs::Twist twist;
    if (target_center_angle_abs < 0.2) {
      twist.linear.x = 0.7;
    }
    twist.angular.z = -target_center_angle;
    this->cmdVelPublisher_.publish(twist);

    auto target_relative_x = target_min_range * cosf(target_center_angle);
    auto target_relative_y = target_min_range * sinf(target_center_angle);

    visualization_msgs::Marker target_marker;
    target_marker.header.frame_id = "base_laser";
    target_marker.header.stamp = ros::Time();
    target_marker.ns = "lab/laserTracker";
    target_marker.type = visualization_msgs::Marker::CYLINDER;
    target_marker.pose.position.x = target_relative_x;
    target_marker.pose.position.y = target_relative_y;
    target_marker.pose.position.z = 0;
    target_marker.scale.x = 0.5;
    target_marker.scale.y = 0.5;
    target_marker.scale.z = 1;
    target_marker.color.a = 1;
    target_marker.color.r = 1;
    target_marker.color.g = 1;
    target_marker.color.b = 1;
    this->vizPublisher_.publish(target_marker);
  }

public:
  explicit LaserTracker(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle) {
    std::string laserSubscriberTopic;
    if (!nodeHandle.getParam("laser_scan_topic", laserSubscriberTopic)) {
      ROS_ERROR("Could not find 'laser_scan_topic' parameter");
    }
    int laserSubscriberQueue;
    if (!nodeHandle.getParam("laser_scan_queue", laserSubscriberQueue)) {
      ROS_ERROR("Could not find 'laser_scan_queue' parameter");
    }
    this->laserSubscriber_ = this->nodeHandle_
                                 .subscribe(laserSubscriberTopic, laserSubscriberQueue,
                                            &LaserTracker::laserScanCallback, this);

    std::string cmdVelPublisherTopic;
    if (!nodeHandle.getParam("cmdVel_topic", cmdVelPublisherTopic)) {
      ROS_ERROR("Could not find 'cmdVel_topic' parameter");
    }
    int cmdVelPublisherQueue;
    if (!nodeHandle.getParam("cmdVel_queue", cmdVelPublisherQueue)) {
      ROS_ERROR("Could not find 'cmdVel_queue' parameter");
    }
    this->cmdVelPublisher_ = this->nodeHandle_
                                 .advertise<geometry_msgs::Twist>(cmdVelPublisherTopic,
                                                                  cmdVelPublisherQueue);
    this->vizPublisher_ = this->nodeHandle_
                              .advertise<visualization_msgs::Marker>("visualization_marker", 0);
  }

  virtual ~LaserTracker() = default;
};

} /* namespace lab */
