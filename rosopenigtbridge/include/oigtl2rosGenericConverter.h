#ifndef ROS2OIGTLGENERICCONVERTER_H
#define ROS2OIGTLGENERICCONVERTER_H

#include <ros/ros.h>
#include <string>
#include <igtlServerSocket.h>

namespace oigtl2ros
{

class GenericConverter
{
public:
    GenericConverter();
    virtual ~GenericConverter();

protected:

    //virtual void Callback(){;}

    ros::NodeHandle mn;
    ros::Publisher mPub;
    igtl::ServerSocket::Pointer mSocket;

};
} //namespace
#endif // ROS2OIGTLGENERICCONVERTER_H
