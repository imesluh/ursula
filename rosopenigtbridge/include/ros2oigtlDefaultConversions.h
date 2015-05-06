#ifndef ROS2OGITLDEFAULTCONVERSIONS_H
#define ROS2OGITLDEFAULTCONVERSIONS_H

//ROS Types
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <shape_msgs/Mesh.h>
#include <std_msgs/Bool.h>

//OpenIGT Link Types
#include <igtlTransformMessage.h>
#include <igtlPositionMessage.h>
#include <igtlImageMessage.h>
#include <igtlImageMetaMessage.h>
#include <igtlPolyDataMessage.h>

#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"


namespace ros2oigtl
{


//Transformations
void TransformToTransform(const geometry_msgs::TransformStamped::ConstPtr &in, igtl::TransformMessage::Pointer out, double scaling = 1.);
void TransformToTransform(igtl::TransformMessage::Pointer in, geometry_msgs::TransformStamped &out);

//Pose & Quaternions

void QTransToTransform(igtl::PositionMessage::Pointer in, geometry_msgs::TransformStamped &out);
void TransformToQTrans(const geometry_msgs::TransformStamped::ConstPtr &in, igtl::PositionMessage::Pointer out);

//Images ToDo

void ImageToImage(igtl::ImageMessage::Pointer in, sensor_msgs::Image &out);
void ImageToImage(const sensor_msgs::Image::ConstPtr& in,igtl::ImageMessage::Pointer out);

//polydata
void MeshToMesh(igtl::PolyDataMessage::Pointer in, shape_msgs::Mesh &out);
void MeshToMesh(const shape_msgs::Mesh::ConstPtr &in, igtl::PolyDataMessage::Pointer out);

}


#endif // ROS2OGITLDEFAULTCONVERSIONS_H
