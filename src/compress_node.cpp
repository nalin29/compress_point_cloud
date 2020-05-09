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
class imageSub
{
public:
    imageSub(ros::NodeHandle n, std::string &t1, std::string &t2, cv::VideoWriter rgbVid, FILE *f) : it_(n), rgb_image(it_, t1, 1), depth_image(it_, t2, 1), sync(MySyncPolicy(10), rgb_image, depth_image), rgbVideo(rgbVid), fp(f)
    {
        sync.registerCallback(boost::bind(&imageSub::callback, this, _1, _2));
    }
    void callback(const sensor_msgs::ImageConstPtr &rgbImage, const sensor_msgs::ImageConstPtr &depthImage)
    {
        // compression methods run here should be moved to a sepeate cpp for running

        cv_bridge::CvImagePtr cv_ptr1 = cv_bridge::toCvCopy(rgbImage, sensor_msgs::image_encodings::BGR8);
        cv::Mat rgb = cv_ptr1->image;
        cv_bridge::CvImagePtr cv_ptr2 = cv_bridge::toCvCopy(depthImage, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::Mat depth = cv_ptr2->image;
        //cv::imshow("depth", depth);
        if (!rgb.data)
            ROS_INFO("Incorrect data type for RGB");
        if (!depth.data)
            ROS_INFO("Incorrect data type for depth");
        rgbVideo.write(rgb);
        compressDepth(depth, fp);
    }
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
    void byteToMat(char *arr)
    {
        cv::Mat test(480, 640, CV_32FC1, arr, cv::Mat::AUTO_STEP);
        //cv::imshow("test", test);
        //cv::waitKey(1);
    }
    void compressDepth(cv::Mat mat, FILE *fp){
        std::vector<float> srcVec = matToByte(mat);
        size_t srcSize = srcVec.size() * sizeof(float);
        char *srcArr = (char *)&srcVec[0];
        maxDest_size = LZ4F_compressFrameBound(srcSize, NULL);
        dest = new char[maxDest_size];
        size_t out = LZ4F_compressFrame(dest, maxDest_size, srcArr, srcSize, NULL);
        //ROS_INFO("Size total: %Ld", srcSize);
        fwrite(dest, 1, out, fp);
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
    ROS_INFO("RGB codec: %s", codec.c_str());
    ROS_INFO("RGB File: %s", rgbFile.c_str());
    ROS_INFO("Depth File: %s", depthFile.c_str());
    static cv::VideoWriter rgbVideo(rgbFile.c_str(), cv::VideoWriter::fourcc(codec[0], codec[1], codec[2], codec[3]), fps, cv::Size(width, height));
    FILE *fp = fopen(depthFile.c_str(), "w+");
    imageSub im(n, rgbImageNode, depthImageNode, rgbVideo, fp);
    ros::spin();
    rgbVideo.release();
    fclose(fp);
    return 0;
}
