#include "ros/ros.h"
#include "std_msgs/String.h"
#include "igtlOSUtil.h"
#include "igtlClientSocket.h"
#include "ros2oigtlDefaultConversions.h"


void meshCallback(const shape_msgs::Mesh::ConstPtr& msg)
{
    ros::NodeHandle nLocal;

    igtl::PolyDataMessage::Pointer oigtlPolydataMsg = igtl::PolyDataMessage::New();
    ros2oigtl::MeshToMesh(msg, oigtlPolydataMsg );


    ROS_INFO("Recieved Transformatipon");
    //send message
    igtl::ClientSocket::Pointer socket;
    socket = igtl::ClientSocket::New();
    std::string server;
    nLocal.getParam("OpenIGTLinkServerIP", server);
    int r;
    if(server.empty())
        r = socket->ConnectToServer("127.0.0.1", 18944);
    else
        r = socket->ConnectToServer(server.c_str(), 18944);

    if (r != 0)
    {
        //
        // do error handling
        //
        ROS_ERROR("Can't connect to OpenIGT Port.");
    }

    oigtlPolydataMsg->Pack();

    socket->Send(oigtlPolydataMsg->GetPackPointer(), oigtlPolydataMsg->GetPackSize());

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
    ros::init(argc, argv, "ros2oigtlMesh");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe(topicName, 100, meshCallback);


    ros::spin();

    return 0;
}

