#include "ros/ros.h"
#include "std_msgs/String.h"
#include "igtlOSUtil.h"
#include "igtlClientSocket.h"
#include "ros2oigtlDefaultConversions.h"
#include "ros2oigtlGenericConverter.h"
//

class PoseConverter : public ros2oigtl::GenericConverter
{
public:
    PoseConverter(std::string topicName);
    ~PoseConverter();

protected:

    void Callback(const geometry_msgs::TransformStamped::ConstPtr& msg);

};

PoseConverter::PoseConverter(std::string topicName)
{

    ROS_INFO("Subscribing topic ...");
    mSub = mn.subscribe(topicName, 100, &PoseConverter::Callback,  this);

    ROS_INFO("Starting server ...");
    m_ServerSocket= igtl::ServerSocket::New();
    std::string port;
    
    int r = m_ServerSocket->CreateServer(18700);
    
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

PoseConverter::~PoseConverter()
{
    m_Socket->CloseSocket();

}

void PoseConverter::Callback(const geometry_msgs::TransformStamped::ConstPtr& msg)
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

    igtl::PositionMessage::Pointer oigtlPositionMsg = igtl::PositionMessage::New();
    ros2oigtl::TransformToQTrans(msg, oigtlPositionMsg );
    oigtlPositionMsg->Pack();
    m_Socket->Send(oigtlPositionMsg->GetPackPointer(), oigtlPositionMsg->GetPackSize());
    ROS_INFO("Transformation is on the way.");


}



int main(int argc, char **argv)
{
    if (argc != 2) // check number of arguments
    {
        // If not correct, print usage
        std::cerr << "Usage: " << argv[0] << " <TopicName>"    << std::endl;
        exit(0);

    }
    ros::init(argc, argv, "ros2oigtlPose");
    std::string topicName = argv[1];
    PoseConverter pc(topicName);
    ros::AsyncSpinner spinner(1);

    spinner.start();
    ros::waitForShutdown();

    return 0;
}
