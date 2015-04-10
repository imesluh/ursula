#include <ros2oigtlbridge.h>

#include "ros2oigtlDefaultConversions.h"
#include "ros2oigtlbridge.h"

namespace ros2oigtl
{

template<typename T>
Ros2OIGTLBridge<T>::Ros2OIGTLBridge(std::string topicName)
{
    mTopicName = topicName;
    mSubscriber = mNodeHandle.subscribe<T>(mTopicName, 100, Ros2OIGTLBridge<T>::GenericCallback,this);


}

template<typename T>
Ros2OIGTLBridge<T>::~Ros2OIGTLBridge()
{


}

template<typename T>
void Ros2OIGTLBridge<T>::SetTopic(std::string topic)
{

    mTopicName = topic;
}

template<typename T>
void Ros2OIGTLBridge<T>::GenericCallback(const T &msg)
{



}
}
