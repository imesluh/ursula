
#include "ros2oigtlDefaultConversions.h"

void TransformToTransform(geometry_msgs::TransformStamped &in, igtl::TransformMessage::Pointer out, double scaling = 1)
{
    //We need to transform geometrymsg (Quaternion+CenterPoint) to a 4x4-Matrix

    out = igtl::TransformMessage::New();
    igtl::Matrix4x4 m;

    //Get data
    geometry_msgs::Quaternion q = in.transform.rotation;
    geometry_msgs::Vector3 center = in.transform.translation;

    //Now transform
    double sqw = q.w*q.w;
    double sqx = q.x*q.x;
    double sqy = q.y*q.y;
    double sqz = q.z*q.z;
    m[0][0] = sqx - sqy - sqz + sqw; // since sqw + sqx + sqy + sqz =1
    m[1][1] = -sqx + sqy - sqz + sqw;
    m[2][2] = -sqx - sqy + sqz + sqw;

    double tmp1 = q.x*q.y;
    double tmp2 = q.z*q.w;
    m[0][1] = 2.0 * (tmp1 + tmp2);
    m[1][0] = 2.0 * (tmp1 - tmp2);

    tmp1 = q.x*q.z;
    tmp2 = q.y*q.w;
    m[0][2] = 2.0 * (tmp1 - tmp2);
    m[2][0] = 2.0 * (tmp1 + tmp2);

    tmp1 = q.y*q.z;
    tmp2 = q.x*q.w;
    m[1][2] = 2.0 * (tmp1 + tmp2);
    m[2][1] = 2.0 * (tmp1 - tmp2);

    double a1,a2,a3;
    a1 = center.x;
    a2 = center.y;
    a3 = center.z;

    m[0][3] = a1 - a1 * m[0][0] - a2 * m[0][1] - a3 * m[0][2];
    m[1][3] = a2 - a1 * m[1][0] - a2 * m[1][1] - a3 * m[1][2];
    m[2][3] = a3 - a1 * m[2][0] - a2 * m[2][1] - a3 * m[2][2];
    m[3][0] = m[3][1] = m[3][2] = 0.0;
    m[3][3] = 1.0;

    out->SetMatrix(m);

    //TODO
    out->SetDeviceName("SomeRosDevice");



}

void TransformToTransform(igtl::TransformMessage::Pointer in, geometry_msgs::TransformStamped &out)
{


}

void QTransToTransform(igtl::PositionMessage::Pointer in, geometry_msgs::TransformStamped &out)
{


}

void TransformToQTrans(geometry_msgs::TransformStamped &in, igtl::PositionMessage::Pointer out)
{


}
