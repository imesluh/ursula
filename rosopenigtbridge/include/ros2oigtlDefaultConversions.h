#ifndef ROS2OGITLDEFAULTCONVERSIONS_H
#define ROS2OGITLDEFAULTCONVERSIONS_H

#include <geometry_msgs/TransformStamped.h>
#include <igtlTransformMessage.h>
#include <igtlPositionMessage.h>
#include <std_msgs/Bool.h>

namespace ros2oigtl
{


//Transformations
void TransformToTransform(geometry_msgs::TransformStamped &in, igtl::TransformMessage::Pointer out, double scaling = 1);
void TransformToTransform(igtl::TransformMessage::Pointer in, geometry_msgs::TransformStamped &out);

//Pose & Quaternions

void QTransToTransform(igtl::PositionMessage::Pointer in, geometry_msgs::TransformStamped &out);
void TransformToQTrans(geometry_msgs::TransformStamped &in, igtl::PositionMessage::Pointer out);

//Images ToDo

//void ImageToImage()

}


#endif // ROS2OGITLDEFAULTCONVERSIONS_H
