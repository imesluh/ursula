#ifndef _ROS2OIGTLBRIDGE_H
#define _ROS2OIGTLBRIDGE_H

#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>


namespace ros2oigtl
{


template<typename T>
class Ros2OIGTLBridge
{
    
public:
    Ros2OIGTLBridge<T>();
    ~Ros2OIGTLBridge<T>();


    
private:


    //void GenericCallback(const T &msg);
    
    ros::Subscriber sub;
       
    
};

} //namespace

#include "../src/ros2oigtlbridge.hxx"



#endif //_ROS2OIGTLBRIDGE_H
