#ifndef _ROS2OIGTLBRIDGE_H
#define _ROS2OIGTLBRIDGE_H

#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>

#include <string>

namespace ros2oigtl
{


template<typename T>
class Ros2OIGTLBridge
{
    
public:
    Ros2OIGTLBridge<T>(std::string topicName);
    ~Ros2OIGTLBridge<T>();

    void SetTopic(std::string topic);
    
private:


    void GenericCallback(const T &msg);
    ros::NodeHandle mNodeHandle;
    ros::Subscriber mSubscriber;
    std::string      mTopicName;
    
};

} //namespace

#include "../src/ros2oigtlbridge.hxx"



#endif //_ROS2OIGTLBRIDGE_H
