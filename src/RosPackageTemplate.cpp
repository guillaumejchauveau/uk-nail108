#include "lab/RosPackageTemplate.hpp"

// STD
#include <string>

namespace lab {

RosPackageTemplate::RosPackageTemplate(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,
                                      &RosPackageTemplate::topicCallback, this);
  serviceServer_ = nodeHandle_.advertiseService("get_average",
                                                &RosPackageTemplate::serviceCallback, this);
  ROS_INFO("Successfully launched node.");
}

RosPackageTemplate::~RosPackageTemplate()
{
}

bool RosPackageTemplate::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  return true;
}

void RosPackageTemplate::topicCallback(const sensor_msgs::Temperature& message)
{

}

bool RosPackageTemplate::serviceCallback(std_srvs::Trigger::Request& request,
                                         std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "The average is ";
  return true;
}

} /* namespace */
