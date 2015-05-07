#ifndef ROS2OIGTLGENERICCONVERTER_H
#define ROS2OIGTLGENERICCONVERTER_H

#include <ros/ros.h>
#include <string>
#include <igtlClientSocket.h>
#include <igtlServerSocket.h>

namespace ros2oigtl
{

class GenericConverter
{
public:
    GenericConverter();
    virtual ~GenericConverter();

protected:

    //virtual void Callback(){;}

    ros::NodeHandle mn;
    ros::Subscriber mSub;
    igtl::ClientSocket::Pointer m_ClientSocket;
    igtl::ServerSocket::Pointer m_ServerSocket;
    igtl::Socket::Pointer m_Socket;

};
} //namespace
#endif // ROS2OIGTLGENERICCONVERTER_H
