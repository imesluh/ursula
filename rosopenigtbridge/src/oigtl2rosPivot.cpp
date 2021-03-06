#include "ros/ros.h"
#include "std_msgs/String.h"
#include "igtlOSUtil.h"
#include "igtlClientSocket.h"
#include "ros2oigtlDefaultConversions.h"
#include "oigtl2rosGenericConverter.h"

#include <string>
//

class TransformationConverter : public oigtl2ros::GenericConverter
{
public:
    TransformationConverter(std::string topicName);
    ~TransformationConverter();

protected:

    int ReceiveTransform(igtl::Socket * socket, igtl::MessageHeader * header, igtl::TransformMessage* transMsg);
    void PublishMessage(igtl::TransformMessage* transMsg);

};


int TransformationConverter::ReceiveTransform(igtl::Socket * socket, igtl::MessageHeader * header, igtl::TransformMessage* transMsg)
{
    ROS_INFO("Receiving TRANSFORM data type.");

    // Create a message buffer to receive transform data
    transMsg->SetMessageHeader(header);
    transMsg->AllocatePack();

    // Receive transform data from the socket
    socket->Receive(transMsg->GetPackBodyPointer(), transMsg->GetPackBodySize());

    // Deserialize the transform data
    // If you want to skip CRC check, call Unpack() without argument.
    int c = transMsg->Unpack(1);

    if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
    {
        // Retrive the transform data
        igtl::Matrix4x4 matrix;
        transMsg->GetMatrix(matrix);
        igtl::PrintMatrix(matrix);
        return 1;
    }

    return 0;

}
TransformationConverter::TransformationConverter(std::string topicName)
{

    mPub = mn.advertise<geometry_msgs::Vector3>(topicName, 1000);   // topicName mit "/" global, sonst auf namespace bezogen
    mSocket = igtl::ServerSocket::New();
    std::string port;
    //    mn.getParam("OpenIGTLinkServerPort", port);
    //    int portInt = atoi(port.c_str());
    int r = mSocket->CreateServer(18757);

    if (r < 0)
    {
        ROS_ERROR("Can't Open OpenIGT Server Port.");
    }
    else
    {
        ROS_INFO("Opened server port.");

    }

    igtl::Socket::Pointer socket;
    geometry_msgs::Vector3 dummyMsg;
    dummyMsg.x = 0.01;
    dummyMsg.y = 0.01;
    dummyMsg.z = 0.01;
    mPub.publish(dummyMsg);
    while (ros::ok())
    {
        //------------------------------------------------------------
        // Waiting for Connection
        socket = mSocket->WaitForConnection(1000);

        if (socket.IsNotNull()) // if client connected
        {

            // Create a message buffer to receive header
            igtl::MessageHeader::Pointer headerMsg;
            headerMsg = igtl::MessageHeader::New();

            //------------------------------------------------------------
            // loop
            for (int i = 0; i < 100; i ++)
            {

                // Initialize receive buffer
                headerMsg->InitPack();

                // Receive generic header from the socket
                int r = socket->Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize());
                if (r == 0)
                {
                    socket->CloseSocket();
                    mPub.publish(dummyMsg);
                }
                if (r != headerMsg->GetPackSize())
                {
                    continue;
                }

                // Deserialize the header
                headerMsg->Unpack();

                // Check data type and receive data body
                if (strcmp(headerMsg->GetDeviceType(), "TRANSFORM") == 0)
                {
                    igtl::TransformMessage::Pointer transMsg;
                    transMsg = igtl::TransformMessage::New();
                    ReceiveTransform(socket, headerMsg, transMsg);
                    PublishMessage(transMsg);

                }
                else
                {
                    // if the data type is unknown, skip reading.
                    ROS_ERROR("Unable to identify message type.");
                    socket->Skip(headerMsg->GetBodySizeToRead(), 0);
                    mPub.publish(dummyMsg);
                }
            }


        }
        else
            mPub.publish(dummyMsg);

    }



    //------------------------------------------------------------
    // Close connection (The example code never reaches to this section ...)

    socket->CloseSocket();
}

TransformationConverter::~TransformationConverter()
{
    mSocket->CloseSocket();

}



void TransformationConverter::PublishMessage(igtl::TransformMessage *transMsg)
{

    ROS_INFO("Publishing it.");
    geometry_msgs::Vector3 out;
    ros2oigtl::TransformToVector3(transMsg, out);

    mPub.publish(out);
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "oigtl2rosPivot");
    TransformationConverter pc("entry_point_origin");
    ros::AsyncSpinner spinner(1);

    spinner.start();
    ros::waitForShutdown();

    return 0;
}
