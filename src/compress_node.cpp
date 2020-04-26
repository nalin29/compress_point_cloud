#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/tfMessage.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/videoio.hpp> 

void callback(const sensor_msgs::ImageConstPtr& rgbImage, const sensor_msgs::CameraInfoConstPtr& rgbInfo, const sensor_msgs::ImageConstPtr& depthImage, const sensor_msgs::CameraInfoConstPtr& depthInfo){
    // compression methods run here should be moved to a sepeate cpp for running
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(rgbImage, sensor_msgs::image_encodings::BGR8);
    cv::VideoWriter rgbVideo("rbgOut.avi", CV_FOURCC('M','J','P','G'), 10, cv::Size(rgbImage->width, rgbImage->height));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "compress_node");
    ros::NodeHandle n;
    std::string rgbImageNode = "camera/rgb/image_raw";
    std::string rgbCamNode = "camera/rgb/camera_info";
    std::string depthImageNode = "camera/depth/image_raw";
    std::string depthCamNode = "camera/depth/camera_info";
    std::string tfNode = "/tf";
    n.param<std::string>("RGB_Image", rgbImageNode, "camera/rgb/image_raw");
    n.param<std::string>("RGB_Cam", rgbCamNode, "camera/rgb/camera_info");
    n.param<std::string>("Depth_Image", depthImageNode, "camera/depth/image_raw");
    n.param<std::string>("Depth_Cam", depthCamNode, "camera/depth/camera_info");
    n.param<std::string>("TF_Data", tfNode, "/tf");
    message_filters::Subscriber<sensor_msgs::Image> rgb_image_sub(n, rgbImageNode, 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> rgb_cam_info(n, rgbCamNode, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_image(n, depthImageNode, 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> depth_cam_info(n, depthCamNode, 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,sensor_msgs::Image, sensor_msgs::CameraInfo> sync(rgb_image_sub, rgb_cam_info, depth_image, depth_cam_info, 5);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));
}