#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/tfMessage.h>

void callback(const sensor_msgs::ImageConstPtr& rgbImage, const sensor_msgs::CameraInfoConstPtr& rgbInfo, const sensor_msgs::ImageConstPtr& depthImage, const sensor_msgs::CameraInfoConstPtr& depthInfo, tf::tfMessageConstPtr& tfData){
    // compression methods run here
}

int main(int argc, char** argv){
    ros::init(argc, argv, "hw4_node");
    ros::NodeHandle n;
    std::string rgbImageNode = "camera/rgb/image_raw";
    std::string rgbCamNode = "camera/rgb/camer_info";
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
    message_filters::Subscriber<sensor_msgs::CameraInfo> depth_came_info(n, depthCamNode, 1);
    message_filters::Subscriber<tf::tfMessage> tf_data(n, tfNode, 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,sensor_msgs::Image, sensor_msgs::CameraInfo, tf::tfMessage> sync(rgb_image_sub, rgb_cam_info, depth_image, depth_came_info, tf_data, 5);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));
}