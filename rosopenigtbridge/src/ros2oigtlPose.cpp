#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ros2oigtlDefaultConversions.h"


void transformCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{

    igtl::PositionMessage::Pointer oigtlPositionMsg;
    ros2oigtl::TransformToQTrans(msg, oigtlPositionMsg );

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




    return 0;
}
