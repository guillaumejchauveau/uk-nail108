#ifndef _CRASHDETECTOR_H_
#define _CRASHDETECTOR_H_
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/SetBool.h>

namespace lab {
class CrashDetector {
protected:
  ros::NodeHandle nodeHandle_;
  ros::ServiceClient startStopServiceClient_;
  ros::Subscriber imuSubscriber_;

  void imuSubscriberCallback(const sensor_msgs::Imu &msg) {
    if (msg.linear_acceleration.y < -2) {
      std_srvs::SetBool::Request request;
      std_srvs::SetBool::Response response;
      request.data = false;
      auto status = this->startStopServiceClient_.call(request, response);

      if (!status || !response.success) {
        ROS_WARN("Stop failed");
      }
    }
  }

public:
  explicit CrashDetector(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle) {
    this->startStopServiceClient_ = this->nodeHandle_
                                        .serviceClient<std_srvs::SetBool>("/lab/start_stop");
    std::string imuSubscriberTopic;
    if (!nodeHandle.getParam("imu_topic", imuSubscriberTopic)) {
      ROS_ERROR("Could not find 'imu_topic' parameter");
    }
    int imuSubscriberQueue;
    if (!nodeHandle.getParam("imu_queue", imuSubscriberQueue)) {
      ROS_ERROR("Could not find 'imu_queue' parameter");
    }
    this->imuSubscriber_ = this->nodeHandle_
                               .subscribe(imuSubscriberTopic, imuSubscriberQueue,
                                          &CrashDetector::imuSubscriberCallback, this);
  }
};

} // namespace lab

#endif //_CRASHDETECTOR_H_
