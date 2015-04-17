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

    mSub = mn.subscribe(topicName, 100, &PoseConverter::Callback,  this);
    mSocket = igtl::ClientSocket::New();
    std::string server;
    mn.getParam("OpenIGTLinkServerIP", server);
    int r;
    if(server.empty())
        r = mSocket->ConnectToServer("127.0.0.1", 18944);
    else
        r = mSocket->ConnectToServer(server.c_str(), 18944);
    if (r != 0)
    {
        //
        // do error handling
        //
        ROS_ERROR("Can't connect to OpenIGT Port.");
    }

}

PoseConverter::~PoseConverter()
{
    mSocket->CloseSocket();

}

void PoseConverter::Callback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
    ROS_INFO("Recieved Transformatipon");
    igtl::PositionMessage::Pointer oigtlPositionMsg = igtl::PositionMessage::New();
    ros2oigtl::TransformToQTrans(msg, oigtlPositionMsg );
    oigtlPositionMsg->Pack();
    mSocket->Send(oigtlPositionMsg->GetPackPointer(), oigtlPositionMsg->GetPackSize());

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
