#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "features.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>

struct ImageProcessor
{
    float imgRows;
    float imgCols;
    bool cv_enabled = false;
    bool boxSended = false;
    ros::Publisher boxSendler;
    ros::Publisher imgPublisher;
    ros::Publisher circlePublisher;
    std::vector<double> cameraDistortion;
    boost::array<double, 9> cameraMatrix;
};

struct ImageProcessor imgProc;

static CirclesMessage cMessage;


// Extract box information from features.cpp by CircleMessage
void cmsg2BoxmultiArray(CirclesMessage& cmsg,
                     std_msgs::Float32MultiArray& boxToSend) {
        std::vector<float> vec1 = {
                                    cmsg.box.left, 
                                    cmsg.box.right, 
                                    cmsg.box.top, 
                                    cmsg.box.bottom,
                                    imgProc.imgRows, imgProc.imgCols
                                    
        };
       boxToSend.data.insert(boxToSend.data.end(), vec1.begin(), vec1.end());
}

// Convert CircleMessage to float multiarray message to send it
void cmsg2multiArray(CirclesMessage& cmsg,
                        std_msgs::Float32MultiArray& msg) {
        std::vector<float> vec1;
        for (size_t i = 0; i != cmsg.inTheBox.size(); ++i) {
            vec1.clear();
            vec1 = {
                    cmsg.circles[i].center.x,
                    cmsg.circles[i].center.y,
                    cmsg.circles[i].size.width,
                    cmsg.circles[i].size.height,
                    cmsg.inTheBox[i]
            };
            msg.data.insert(msg.data.end(), vec1.begin(), vec1.end());
        }
} 


// Get and process the image from camera

void onImage(const sensor_msgs::Image::ConstPtr& image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_8UC3);
        }
        catch	 (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }
   if (imgProc.cv_enabled) {
        // Process image and get target information
        processImage(cv_ptr->image, cMessage);
        
        // Convert circle message from features.cpp and send it
        std_msgs::Float32MultiArray msg;
        std_msgs::Float32MultiArray sendBox;
        // Information about the box sends only 1 time
        // Convert message

//        if (!imgProc.boxSended) {
            imgProc.imgRows = cv_ptr->image.rows;
            imgProc.imgCols = cv_ptr->image.cols;
            cmsg2BoxmultiArray(cMessage,  sendBox);
            imgProc.boxSendler.publish(sendBox);
            imgProc.boxSended = true;
//        } else {
            cmsg2multiArray(cMessage, msg);
//        }
        // Publish message
        imgProc.circlePublisher.publish(msg);
    }

    // Publish image
    imgProc.imgPublisher.publish(cv_ptr->toImageMsg());

}

// Hz
void onCameraInfo(const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    imgProc.cameraDistortion = cam_info->D;
    imgProc.cameraMatrix = cam_info->K;
}

// Enable/disable image processing

void onEnable(const std_msgs::Empty& toggle_msg) {
    imgProc.cv_enabled = !imgProc.cv_enabled;
    if (imgProc.cv_enabled) {
        std::cout << "Image processing enabled.\n";
    } else {
        std::cout << "Image processing disabled.\n";
    }
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "imgproc");
    ros::NodeHandle node;
    
    // M button
    ros::Subscriber enableSub = 
            node.subscribe("cv/enable", 1, onEnable);
    
    imgProc.imgPublisher = 
            node.advertise<sensor_msgs::Image>("/out/image", 5);
    
    imgProc.boxSendler = 
            node.advertise<std_msgs::Float32MultiArray>("box", 1);
    imgProc.circlePublisher = 
            node.advertise<std_msgs::Float32MultiArray>("target", 5);
    
    // Getting the image from camera
    ros::Subscriber sub1 = 
            node.subscribe("/in/image", 5, onImage);
    ros::Subscriber sub2 = 
            node.subscribe("/ardrone/image_raw", 5, onImage);
    ros::Subscriber sub3 = 
            node.subscribe("/ardrone/camera_info", 5, onCameraInfo);

    ros::spin();
    //int count = 0;
    //++count;


    return 0;
}
