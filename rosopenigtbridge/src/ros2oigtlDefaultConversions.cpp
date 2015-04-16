
#include "ros2oigtlDefaultConversions.h"



void ros2oigtl::TransformToTransform(const geometry_msgs::TransformStamped::ConstPtr &in, igtl::TransformMessage::Pointer out, double scaling)
{
    //We need to transform geometrymsg (Quaternion+CenterPoint) to a 4x4-Matrix

    out = igtl::TransformMessage::New();
    igtl::Matrix4x4 m;

    //Get data
    geometry_msgs::Quaternion q = in->transform.rotation;
    geometry_msgs::Vector3 center = in->transform.translation;

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


    //Setup Header
    igtl::TimeStamp::Pointer stamp = igtl::TimeStamp::New();
    stamp->SetTime((double)(in->header.stamp.toNSec()));
    out->SetTimeStamp(stamp);
    //TODO
    out->SetDeviceName("SomeRosDevice");


}

void ros2oigtl::TransformToTransform(igtl::TransformMessage::Pointer in, geometry_msgs::TransformStamped &out)
{


}

void ros2oigtl::QTransToTransform(igtl::PositionMessage::Pointer in, geometry_msgs::TransformStamped &out)
{
    if(!in)
        return;

    //get Quaternion
    float* ox = new float(.0);
    float* oy = new float(.0);
    float* oz = new float(.0);
    float* ow = new float(.0);

    in->GetQuaternion(ox,oy,oz,ow);

    out.transform.rotation.x = (double)(*ox);
    out.transform.rotation.y = (double)(*oy);
    out.transform.rotation.z = (double)(*oz);
    out.transform.rotation.w = (double)(*ow);

    //get vec
    float* x = new float(.0);
    float* y = new float(.0);
    float* z = new float(.0);

    in->GetPosition(x, y, z);

    out.transform.translation.x = (double)(*x);
    out.transform.translation.y = (double)(*y);
    out.transform.translation.z = (double)(*z);

}

void ros2oigtl::TransformToQTrans(const geometry_msgs::TransformStamped::ConstPtr &in, igtl::PositionMessage::Pointer out)
{
  //   out = igtl::PositionMessage::New();

    float position[3];
    float quaternion[4];

    position[0] = (float)in->transform.translation.x;
    position[1] = (float)in->transform.translation.y;
    position[2] = (float)in->transform.translation.z;

    quaternion[0] = (float)in->transform.rotation.x;
    quaternion[1] = (float)in->transform.rotation.y;
    quaternion[2] = (float)in->transform.rotation.z;
    quaternion[3] = (float)in->transform.rotation.w;

    out->SetPosition(position);
    out->SetQuaternion(quaternion);

    //Setup Header
    igtl::TimeStamp::Pointer stamp = igtl::TimeStamp::New();
    stamp->SetTime((double)(in->header.stamp.toNSec()));
    out->SetTimeStamp(stamp);
    //TODO
    out->SetDeviceName("SomeRosDevice");

}

void ros2oigtl::ImageToImage(igtl::ImageMessage::Pointer in, sensor_msgs::Image &out)
{
    if(!in)
        return;

    int width, height, depth;

    in->GetDimensions(width, height, depth);

    //check for 2d
    if(depth >1)
        return;

    out.width = width;
    out.height = height;



}

void ros2oigtl::ImageToImage(const sensor_msgs::Image::ConstPtr& in,igtl::ImageMessage::Pointer out)
{

    //Setup Header
    igtl::TimeStamp::Pointer stamp = igtl::TimeStamp::New();
    stamp->SetTime((double)(in->header.stamp.toNSec()));
    out->SetTimeStamp(stamp);
    //TODO
    out->SetDeviceName("SomeRosDevice");
}

void ros2oigtl::MeshToMesh(igtl::PolyDataMessage::Pointer in, shape_msgs::Mesh &out)
{


}

void ros2oigtl::MeshToMesh(const shape_msgs::Mesh::ConstPtr &in, igtl::PolyDataMessage::Pointer out)
{

    //Setup Header *shape_msgs::Mesh has no header*


    //TODO
    out->SetDeviceName("SomeRosDevice");
}
