
#include <ros/ros.h>
#include "ros2oigtlDefaultConversions.h"

#include <eigen3/Eigen/Geometry>



void ros2oigtl::TransformToTransform(const geometry_msgs::TransformStamped::ConstPtr &in, igtl::TransformMessage::Pointer out, std::string deviceName, double scaling)
{
    //We need to transform geometrymsg (Quaternion+CenterPoint) to a 4x4-Matrix
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

    m[0][3] = a1*1000.;// - a1 * m[0][0] - a2 * m[0][1] - a3 * m[0][2];
    m[1][3] = a2*1000.;// - a1 * m[1][0] - a2 * m[1][1] - a3 * m[1][2];
    m[2][3] = a3*1000.;// - a1 * m[2][0] - a2 * m[2][1] - a3 * m[2][2];


    m[3][0] = m[3][1] = m[3][2] = 0.0;
    m[3][3] = 1.0;

    out->SetMatrix(m);


    //Setup Header
    igtl::TimeStamp::Pointer stamp = igtl::TimeStamp::New();
    stamp->SetTime((double)(in->header.stamp.toNSec()));
    out->SetTimeStamp(stamp);
    //TODO
    out->SetDeviceName(deviceName.c_str());

}

void ros2oigtl::TransformToTransform2(const geometry_msgs::TransformStamped::ConstPtr &in, igtl::TransformMessage::Pointer out, std::string deviceName, double scaling)
{
    //We need to transform geometrymsg (Quaternion+CenterPoint) to a 4x4-Matrix
    igtl::Matrix4x4 m;

    //Get data
    geometry_msgs::Quaternion q = in->transform.rotation;
    geometry_msgs::Vector3 center = in->transform.translation;

    Eigen::Affine3d TMATRIX = Eigen::Translation3d(Eigen::Vector3d(center.x*1000, center.y*1000, center.z*1000)) * Eigen::Quaterniond((double)q.w, (double)q.x, (double)q.y, (double)q.z);

    std::cout << TMATRIX.matrix();

    m[0][0] = TMATRIX(0,0);
    m[0][1] = TMATRIX(0,1);
    m[0][2] = TMATRIX(0,2);
    m[0][3] = TMATRIX(0,3);

    m[1][0] = TMATRIX(1,0);
    m[1][1] = TMATRIX(1,1);
    m[1][2] = TMATRIX(1,2);
    m[1][3] = TMATRIX(1,3);

    m[2][0] = TMATRIX(2,0);
    m[2][1] = TMATRIX(2,1);
    m[2][2] = TMATRIX(2,2);
    m[2][3] = TMATRIX(2,3);

    m[3][0] = TMATRIX(3,0);
    m[3][1] = TMATRIX(3,1);
    m[3][2] = TMATRIX(3,2);
    m[3][3] = TMATRIX(3,3);

    std::cout << "\n";
    std::cout << "*************************************************************\n";
    for(int i = 0;i< 4; i++)
    {
        for(int z = 0; z<4; z++)
        {
            std::cout << m[i][z] << " ";
        }
        std::cout << "\n";
    }

    std::cout << "\n";
    std::cout << "*************************************************************";
    std::cout << "\n";
//    tf::Quaternion tfQuat;
//    tfQuat.x = q.x;
//    tfQuat.y = q.y;
//    tfQuat.z = q.z;
//    tfQuat.w = q.w;

//    tf::Matrix3x3 tfMat(tfQuat);

//    m[0][0] = tfMat[0][0];
//    m[0][1] = tfMat[0][1];
//    m[0][2] = tfMat[0][2];


//    m[1][0] = tfMat[1][0];
//    m[1][1] = tfMat[1][1];
//    m[1][2] = tfMat[1][2];

//    m[2][0] = tfMat[2][0];
//    m[2][1] = tfMat[2][1];
//    m[2][2] = tfMat[2][2];

//    m[]


    out->SetMatrix(m);
    //Setup Header
    igtl::TimeStamp::Pointer stamp = igtl::TimeStamp::New();
    stamp->SetTime((double)(in->header.stamp.toNSec()));
    out->SetTimeStamp(stamp);
    //TODO
    out->SetDeviceName(deviceName.c_str());

}

void ros2oigtl::TransformToVector3(igtl::TransformMessage::Pointer in, geometry_msgs::Vector3 &out)
{
    igtl::Matrix4x4 m;
    in->GetMatrix(m);
//    tf::Vector3 origin;
//    origin.setValue(static_cast<double>(m[0][3]),static_cast<double>(m[1][3]),static_cast<double>(m[2][3]));

    out.x = m[0][3]/1000.;
    out.y = m[1][3]/1000.;
    out.z = m[2][3]/1000.;
}

void ros2oigtl::TransformToTransform(const std_msgs::Float64MultiArray &in, igtl::TransformMessage::Pointer out, std::string deviceName)
{
    if(in.data.size() < 16)
        return;
    out = igtl::TransformMessage::New();
    igtl::Matrix4x4 m;

    m[0][0] = in.data[0];
    m[0][1] = in.data[1];
    m[0][2] = in.data[2];
    m[0][3] = in.data[3];

    m[1][0] = in.data[4];
    m[1][1] = in.data[5];
    m[1][2] = in.data[6];
    m[1][3] = in.data[7];

    m[2][0] = in.data[8];
    m[2][1] = in.data[9];
    m[2][2] = in.data[10];
    m[2][3] = in.data[11];

    m[3][0] = in.data[12];
    m[3][1] = in.data[13];
    m[3][2] = in.data[14];
    m[3][3] = in.data[15];

    out->SetMatrix(m);
    out->SetDeviceName(deviceName.c_str());

//    igtl::TimeStamp::Pointer stamp = igtl::TimeStamp::New();
//    in->GetTimeStamp(stamp);

//    unsigned int nsec = stamp->GetNanosecond();
//    out.header.stamp.fromNSec(nsec);
}


void ros2oigtl::TransformToTransform(igtl::TransformMessage::Pointer in, geometry_msgs::TransformStamped &out)
{
    //Get data
    geometry_msgs::Quaternion q = out.transform.rotation;


    igtl::Matrix4x4 m;
    in->GetMatrix(m);
    //    tf::Vector3 origin;
    //    origin.setValue(static_cast<double>(m[0][3]),static_cast<double>(m[1][3]),static_cast<double>(m[2][3]));


    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(m[0][0]), static_cast<double>(m[0][1]), static_cast<double>(m[0][2]),
            static_cast<double>(m[1][0]), static_cast<double>(m[1][1]), static_cast<double>(m[1][2]),
            static_cast<double>(m[2][0]), static_cast<double>(m[2][1]), static_cast<double>(m[2][2]));

    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);

    out.transform.translation.x = m[0][3]/1000.;
    out.transform.translation.y = m[1][3]/1000.;
    out.transform.translation.z = m[2][3]/1000.;

    out.transform.rotation.x = tfqt.getX();
    out.transform.rotation.y = tfqt.getY();
    out.transform.rotation.z = tfqt.getZ();
    out.transform.rotation.w = tfqt.getW();

    //Setup Header

    igtl::TimeStamp::Pointer stamp = igtl::TimeStamp::New();
    in->GetTimeStamp(stamp);

    unsigned int nsec = stamp->GetNanosecond();
    out.header.stamp.fromNSec(nsec);
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

void ros2oigtl::TransformToTransform(const geometry_msgs::TransformStamped &in, igtl::TransformMessage::Pointer out, std::string deviceName)
{
    tf::Quaternion quat;
    quat.setX((tfScalar)in.transform.rotation.x);
    quat.setY((tfScalar)in.transform.rotation.y);
    quat.setZ((tfScalar)in.transform.rotation.z);
    quat.setW((tfScalar)in.transform.rotation.w);



    tf::Matrix3x3 rotation(quat);

    igtl::Matrix4x4 m;
    //Set Rotation#
    for(int z = 0; z <3; z++)
    {
        for(int s = 0; s < 3;s++)
        {
            m[z][s] = rotation[z][s];

        }
    }


    //Set translation & Scaling
    m[3][3] = 1;

    m[0][3] = in.transform.translation.x;
    m[1][3] = in.transform.translation.y;
    m[2][3] = in.transform.translation.z;

    out->SetMatrix(m);
    //Setup Header
    igtl::TimeStamp::Pointer stamp = igtl::TimeStamp::New();
    stamp->SetTime((double)(in.header.stamp.toNSec()));
    out->SetTimeStamp(stamp);
    //TODO
    out->SetDeviceName(deviceName.c_str());



}

void ros2oigtl::TransformToQTrans(const geometry_msgs::TransformStamped::ConstPtr &in, igtl::PositionMessage::Pointer out, std::string deviceName)
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
    out->SetDeviceName(deviceName.c_str());

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


    // in->vertices
}
