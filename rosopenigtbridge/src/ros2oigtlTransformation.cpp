#include "ros/ros.h"
#include "std_msgs/String.h"
#include "igtlOSUtil.h"
#include "igtlClientSocket.h"
#include "ros2oigtlDefaultConversions.h"
#include "ros2oigtlGenericConverter.h"
//

class TransformationConverter : public ros2oigtl::GenericConverter
{
public:
    TransformationConverter(std::string topicName, int portNumb, std::string deviceName );
    ~TransformationConverter();

protected:

    void Callback(const geometry_msgs::TransformStamped::ConstPtr& msg);
    std::string m_DeviceName;

};

TransformationConverter::TransformationConverter(std::string topicName, int portNumb, std::string deviceName)
{

    m_DeviceName = deviceName;
    ROS_INFO("Subscribing topic ...");
    mSub = mn.subscribe(topicName, 100, &TransformationConverter::Callback,  this);

    ROS_INFO("Starting server ...");
    m_ServerSocket= igtl::ServerSocket::New();

    
    int r = m_ServerSocket->CreateServer(portNumb);
    
    if (r < 0)
    {
        ROS_ERROR("Can't Open OpenIGT Server Port.");
    }
    else
    {
        ROS_INFO("Opened server port.");

    }

    // Waiting for Connection
    m_Socket = m_ServerSocket->WaitForConnection(1000);
    if (m_Socket.IsNotNull()) // if client connected
    {
        ROS_INFO("Socket established. Waiting for topics to send ...");
    }

    
}

TransformationConverter::~TransformationConverter()
{
    m_Socket->CloseSocket();

}

void TransformationConverter::Callback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
    ROS_INFO("Recieved Transformation");

    if(m_Socket.IsNull())
    {
        ROS_WARN("Socket closed, trying to reopen it ...");
        m_Socket = m_ServerSocket->WaitForConnection(1000);
    }
    if (m_Socket.IsNotNull()) // if client connected
    {
        ROS_INFO("Open.");
    }
    else
    {
        ROS_ERROR("Unable to open.");
        return;
    }

    igtl::TransformMessage::Pointer oigtlTransformMsg = igtl::TransformMessage::New();
    ros2oigtl::TransformToTransform(msg, oigtlTransformMsg, m_DeviceName );
    oigtlTransformMsg->Pack();
    m_Socket->Send(oigtlTransformMsg->GetPackPointer(), oigtlTransformMsg->GetPackSize());
    ROS_INFO("Transformation is on the way.");


}



int main(int argc, char **argv)
{
    if (argc != 4) // check number of arguments
    {
        // If not correct, print usage
        std::cerr << "Usage: " << argv[0] << " <TopicName> <PORT> <DeviceName>"  << std::endl;
        exit(0);

    }
    ros::init(argc, argv, "ros2oigtlPose");
    std::string topicName = argv[1];
    int portNumb = atoi(argv[2]);
    std::string deviceName = argv[3];

    TransformationConverter pc(topicName, portNumb, deviceName);
    ros::AsyncSpinner spinner(1);

    spinner.start();
    ros::waitForShutdown();

    return 0;
}
