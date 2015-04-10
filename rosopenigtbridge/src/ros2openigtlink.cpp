#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ros2oigtlbridge.h"

#include <geometry_msgs/Vector3.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ros2openigtlink");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros2oigtl::Ros2OIGTLBridge<geometry_msgs::Quaternion> bridge(std::string("/mypose"));


  return 0;
}
