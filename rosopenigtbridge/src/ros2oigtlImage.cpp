#include "ros/ros.h"
#include "std_msgs/String.h"
#include "igtlOSUtil.h"
#include "igtlClientSocket.h"
#include "ros2oigtlDefaultConversions.h"


void transformCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
//    ros::NodeHandle nLocal;

//    igtl::PositionMessage::Pointer oigtlPositionMsg = igtl::PositionMessage::New();
//    ros2oigtl::TransformToQTrans(msg, oigtlPositionMsg, );


//    ROS_INFO("Recieved Transformatipon");
//    //send message
//    igtl::ClientSocket::Pointer socket;
//    socket = igtl::ClientSocket::New();
//    std::string server;
//    nLocal.getParam("OpenIGTLinkServerIP", server);
//    int r;
//    if(server.empty())
//        r = socket->ConnectToServer("127.0.0.1", 18944);
//    else
//        r = socket->ConnectToServer(server.c_str(), 18944);

//    if (r != 0)
//    {
//        //
//        // do error handling
//        //
//        ROS_ERROR("Can't connect to OpenIGT Port.");
//    }

//    oigtlPositionMsg->Pack();

//    socket->Send(oigtlPositionMsg->GetPackPointer(), oigtlPositionMsg->GetPackSize());

}

int main(int argc, char **argv)
{
    if (argc != 2) // check number of arguments
    {
        // If not correct, print usage
        std::cerr << "Usage: " << argv[0] << " <TopicName>"    << std::endl;
        exit(0);
    }

    std::string topicName = argv[1];
    ros::init(argc, argv, "ros2oigtlPose");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe(topicName, 100, transformCallback);


    ros::spin();

    return 0;
}


