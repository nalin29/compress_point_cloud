#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/tfMessage.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/String.h>
#include <roslz4/lz4s.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <opencv2/imgcodecs.hpp>
class imageSub
{  
    public:
    imageSub(ros::NodeHandle n, std::string &t1, std::string &t2, cv::VideoWriter rgbVid, roslz4_stream depth, FILE* f) : it_(n), rgb_image(it_, t1, 1), depth_image(it_, t2, 1), sync(MySyncPolicy(10),rgb_image, depth_image), rgbVideo(rgbVid), depthStream(depth), fp(f)
    {
        sync.registerCallback(boost::bind(&imageSub::callback, this, _1, _2));
    }
    void callback(const sensor_msgs::ImageConstPtr &rgbImage, const sensor_msgs::ImageConstPtr &depthImage)
    {
        // compression methods run here should be moved to a sepeate cpp for running
        

        cv_bridge::CvImagePtr cv_ptr1 = cv_bridge::toCvCopy(rgbImage, sensor_msgs::image_encodings::BGR8);
        cv::Mat image1 = cv_ptr1->image;
        cv_bridge::CvImagePtr cv_ptr2 = cv_bridge::toCvCopy(depthImage, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::Mat image2 = cv_ptr2->image;
        if(!image1.data)
            ROS_INFO("Incorrect data type for RGB");
        if(!image2.data)
            ROS_INFO("Incorrect data type for depth");
        cv::imshow("RGB Image", image1);
        cv::imshow("Depth Image", image2);
        rgbVideo.write(image1);
        std::vector<uchar> buf;
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(100);
        cv::imencode(".jpg", image2, buf, compression_params);
        char *out = (char *)malloc(buf.size());
        depthStream.input_next = (char *)(&buf[0]);
        depthStream.output_next = out;
        depthStream.output_left = buf.size();
        depthStream.input_left = buf.size();
        roslz4_compress(&depthStream, ROSLZ4_FINISH);
        fwrite(out, 1, buf.size(), fp);
        free(out);
        cv::waitKey(1);
    }

private:
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter rgb_image;
    image_transport::SubscriberFilter depth_image;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync;
    cv::VideoWriter rgbVideo;
    cv::VideoWriter depthVideo;
    roslz4_stream depthStream;
    FILE* fp;
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "compress_node");
    ros::NodeHandle n("~");
    std::string rgbImageNode = "/camera/rgb/image_color";
    n.param<std::string>("/compress_node/video",rgbImageNode, "/camera/rgb/image_color");
    std::string depthImageNode = "/camera/depth/image_raw";
    n.param<std::string>("/compress_node/depth",depthImageNode, "/camera/depth/image_raw");
    int width;
    n.param("/compress_node/width", width, 640);
    int height;
    n.param("/compress_node/height", height, 480);
    std::string codec;
    n.param<std::string>("/compress_node/codec",codec,"MJPG");
    ROS_INFO("RGB codec: %c%c%c%c", codec[0], codec[1], codec[2], codec[3]);
    static cv::VideoWriter rgbVideo("RGBout.avi", cv::VideoWriter::fourcc(codec[0], codec[1], codec[2], codec[3]), 30, cv::Size(640, 480));
    //static cv::VideoWriter depthVideo("Depthout.avi", cv::VideoWriter::fourcc('F','F','V','1'), 30, cv::Size(640, 480), false);
    FILE* fp = fopen("test", "w+");
    roslz4_stream depthStream;
    int ret = roslz4_compressStart(&depthStream, 4);
    imageSub im(n, rgbImageNode, depthImageNode, rgbVideo, depthStream, fp);
    ros::spin();
    rgbVideo.release();
    roslz4_compressEnd(&depthStream);
    return 0;
}