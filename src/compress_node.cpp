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
class pointCloudBroadcaster
{
public:
    pointCloudBroadcaster(ros::NodeHandle n, std::string &t1, std::string &t2, cv::VideoWriter rgbVid, FILE *f) : it_(n), rgb_image(it_, t1, 1), depth_image(it_, t2, 1), sync(MySyncPolicy(10), rgb_image, depth_image), rgbVideo(rgbVid), fp(f)
    {
        // sync using syncpolicy and call callback for compression
        sync.registerCallback(boost::bind(&pointCloudBroadcaster::callback, this, _1, _2));
    }
    // compresses and writes data
    void callback(const sensor_msgs::ImageConstPtr &rgbImage, const sensor_msgs::ImageConstPtr &depthImage)
    {
        // get images with hardcoded encoding values
        cv_bridge::CvImagePtr cv_ptr1 = cv_bridge::toCvCopy(rgbImage, sensor_msgs::image_encodings::BGR8);
        cv::Mat rgb = cv_ptr1->image;
        cv_bridge::CvImagePtr cv_ptr2 = cv_bridge::toCvCopy(depthImage, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::Mat depth = cv_ptr2->image;
        // ensure data is correct type
        if (!rgb.data)
            ROS_INFO("Incorrect data type for RGB");
        if (!depth.data)
            ROS_INFO("Incorrect data type for depth");
        // write compressed data
        rgbVideo.write(rgb);
        compressDepth(depth, fp);
    }
    //converts a cv mat to a byte vector
    std::vector<float> matToByte(cv::Mat mat)
    {
        std::vector<float> array;
        if (mat.isContinuous())
            array.assign((float *)mat.data, (float *)mat.data + mat.total());
        else
            for (int i = 0; i < mat.rows; ++i)
                array.insert(array.end(), mat.ptr<float>(i), mat.ptr<float>(i) + mat.cols);
        return array;
    }
    // code to test if byte vector init properly
    void byteToMat(char *arr)
    {
        cv::Mat test(480, 640, CV_32FC1, arr, cv::Mat::AUTO_STEP);
        cv::imshow("test", test);
        cv::waitKey(1);
    }
    // compresses matrix and writes to file using lz4
    void compressDepth(cv::Mat mat, FILE *fp){
        // get size and create byte array from mat
        std::vector<float> srcVec = matToByte(mat);
        size_t srcSize = mat.rows*mat.cols*mat.elemSize();
        char *srcArr = (char *)&srcVec[0];
        // get max size of compressed data for compressed data buffer
        maxDest_size = LZ4F_compressFrameBound(srcSize, NULL);
        dest = new char[maxDest_size];
        // compresses data and stores in dest out is the number of bytes to write
        size_t out = LZ4F_compressFrame(dest, maxDest_size, srcArr, srcSize, NULL);
        // write bytes to file
        fwrite(dest, 1, out, fp);
        // delete dest buffer
        delete(dest);
    }

private:
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter rgb_image;
    image_transport::SubscriberFilter depth_image;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync;
    cv::VideoWriter rgbVideo;
    cv::VideoWriter depthVideo;
    FILE *fp;
    size_t maxDest_size;
    char *dest;
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "compress_node");
    ros::NodeHandle n("~");
    //parameter init
    std::string rgbImageNode = "/camera/rgb/image_color";
    n.param<std::string>("/compress_node/video", rgbImageNode, "/camera/rgb/image_rect_color");
    std::string depthImageNode = "/camera/depth/image";
    n.param<std::string>("/compress_node/depth", depthImageNode, "/camera/depth_registered/sw_registered/image_rect");
    int width;
    n.param("/compress_node/width", width, 640);
    int height;
    n.param("/compress_node/height", height, 480);
    std::string codec;
    n.param<std::string>("/compress_node/codec", codec, "MJPG");
    int fps;
    n.param<int>("/compress_node/fps", fps, 30);
    std::string rgbFile;
    n.param<std::string>("/compress_node/rgb_file", rgbFile, "RGBout.avi");
    std::string depthFile;
    n.param<std::string>("/compress_node/depth_file", depthFile, "depth_data");
    //debug status
    ROS_INFO("RGB codec: %s", codec.c_str());
    ROS_INFO("RGB File: %s", rgbFile.c_str());
    ROS_INFO("Depth File: %s", depthFile.c_str());
    // open a videowriter to write to video file
    static cv::VideoWriter rgbVideo(rgbFile.c_str(), cv::VideoWriter::fourcc(codec[0], codec[1], codec[2], codec[3]), fps, cv::Size(width, height));
    // open a file to write lz4 compressed data to
    FILE *fp = fopen(depthFile.c_str(), "w+");
    // run synced image subsrciber
    pointCloudBroadcaster im(n, rgbImageNode, depthImageNode, rgbVideo, fp);
    ros::spin();
    // release buffers
    rgbVideo.release();
    fclose(fp);
    return 0;
}
