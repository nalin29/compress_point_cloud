#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/String.h>
#include <opencv2/imgcodecs.hpp>
#include <lz4frame.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
// code modified form depth_image_proc ros nodelet copyright:
/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*********************************************************************/
class pointCloudBroadcaster
{
public:
    typedef sensor_msgs::PointCloud2 PointCloud;
    pointCloudBroadcaster(ros::NodeHandle n, std::string rgb, std::string depth, std::string info) : it_(n), rgb_image(it_, rgb, 1), depth_image(it_, depth, 1), camInfo(n, info, 1), sync(MySyncPolicy(10), rgb_image, depth_image, camInfo)
    {
        // advertise point cloud and run sync on images and camera_info
        pub_point_cloud_ = n.advertise<PointCloud>("/camera/depth_registered/points", 1);
        // sync data streams and publish point cloud based on them
        sync.registerCallback(boost::bind(&pointCloudBroadcaster::callback, this, _1, _2, _3));
    }
    void callback(const sensor_msgs::ImageConstPtr &rgb_msg_in, const sensor_msgs::ImageConstPtr &depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        // Update camera model
        model_.fromCameraInfo(info_msg);
        sensor_msgs::ImageConstPtr rgb_msg = rgb_msg_in;
        rgb_msg = rgb_msg_in;
        // offset based on rgb image encoding BGR8
        int red_offset, green_offset, blue_offset, color_step;
        red_offset = 2;
        green_offset = 1;
        blue_offset = 0;
        color_step = 3;
        // Allocate new point cloud message
        PointCloud::Ptr cloud_msg(new PointCloud);
        cloud_msg->header = depth_msg->header; // Use depth image time stamp
        cloud_msg->height = depth_msg->height;
        cloud_msg->width = depth_msg->width;
        cloud_msg->is_dense = false;
        cloud_msg->is_bigendian = false;

        // create modifier to step through point cloud and alter it
        sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
        pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        // get center from camera info
        float center_x = model_.cx();
        float center_y = model_.cy();

        // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
        double unit_scaling = 1;
        float constant_x = unit_scaling / model_.fx();
        float constant_y = unit_scaling / model_.fy();
        float bad_point = std::numeric_limits<float>::quiet_NaN();
        // get row pointer and length of row/col of point cloud matrix
        const float *depth_row = reinterpret_cast<const float *>(&depth_msg->data[0]);
        int row_step = depth_msg->step / sizeof(float);
        const uint8_t *rgb = &rgb_msg->data[0];
        int rgb_skip = rgb_msg->step - rgb_msg->width * color_step;

        // initialize iterators over point cloud arr
        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_a(*cloud_msg, "a");

        // loop through point cloud arr
        for (int v = 0; v < int(cloud_msg->height); ++v, depth_row += row_step, rgb += rgb_skip)
        {
            for (int u = 0; u < int(cloud_msg->width); ++u, rgb += color_step, ++iter_x, ++iter_y, ++iter_z, ++iter_a, ++iter_r, ++iter_g, ++iter_b)
            {
                // data expected from rviz in m hence divide by 1000 since kinect in mm
                float depth = depth_row[u]/1000.0F;
                // Fill in XYZ
                *iter_x = (u - center_x) * depth * constant_x;
                *iter_y = (v - center_y) * depth * constant_y;
                *iter_z = depth;
                // Fill in color
                *iter_a = 255;
                *iter_r = rgb[red_offset];
                *iter_g = rgb[green_offset];
                *iter_b = rgb[blue_offset];
            }
        }
        // publish point cloud
        pub_point_cloud_.publish(cloud_msg);
    }
private:
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter rgb_image;
    image_transport::SubscriberFilter depth_image;
    message_filters::Subscriber<sensor_msgs::CameraInfo> camInfo;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync;
    sensor_msgs::CameraInfo cameraInfo;
    image_geometry::PinholeCameraModel model_;
    ros::Publisher pub_point_cloud_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointCloud_node");
    ros::NodeHandle n;
    // broadcast point cloud from these rostopics
    pointCloudBroadcaster pcb(n, "/camera/rgb/image_rect_color", "/camera/depth_registered/sw_registered/image_rect", "/camera/rgb/camera_info");
    ros::spin();
    return 0;
}
