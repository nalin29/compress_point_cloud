#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/tfMessage.h>
#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/String.h>
#include <lz4frame.h>
#include <string>
class CompressBroadcaster
{
	image_transport::ImageTransport it;
	image_transport::Publisher rgbVideo;
	image_transport::Publisher depthVideo;

public:
	CompressBroadcaster(ros::NodeHandle nh) : it(nh)
	{
		rgbVideo = it.advertise("/camera/rgb/image_color", 1);
		depthVideo = it.advertise("/camera/depth/image", 1);
	}
	void publish(cv::Mat rgbImage, cv::Mat depthImage)
	{
		cv::Mat compressImage = rgbImage;
		cv_bridge::CvImage out_msg;
		out_msg.header = std_msgs::Header();
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image = compressImage;

		cv::Mat compressDepthImage = depthImage;
		cv_bridge::CvImage outDepth_msg;
		outDepth_msg.header = std_msgs::Header();
		outDepth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
		outDepth_msg.image = compressDepthImage;
		rgbVideo.publish(out_msg.toImageMsg());
		depthVideo.publish(outDepth_msg.toImageMsg());
	}
};
class SyncedDecompressor
{
public:
	SyncedDecompressor(std::string rgbFile, std::string depthFile, CompressBroadcaster cb) : rgbFile_(rgbFile), depthFile_(depthFile), broadcast_(cb), cap_(rgbFile_)
	{
		fileArr("test");
		read_ = 0;
		if (!cap_.isOpened())
			std::cout << "no video" << std::endl;
		// runs at rate of 30hz
		ros::Rate rate(30.0);
		while (cap_.isOpened())
		{
			cv::Mat frame;
			cap_ >> frame;
			if (frame.empty())
				break;
			cv::Mat depthFrame;
			size_t total = 640 * 480 * depthFrame.elemSize();
			LZ4F_createDecompressionContext(&dctx_, LZ4F_VERSION);
			char * decompressDest = decompress(1228800, fileSize_-read_, compressedSrc_+read_);
			depthFrame = byteToMat(decompressDest);
			broadcast_.publish(frame, depthFrame);
			delete decompressDest;
			// wait for 30hz rate
			rate.sleep();
		}
		cap_.release();
		LZ4F_freeDecompressionContext(dctx_);
	}
	~SyncedDecompressor(){
		delete compressedSrc_;
	}
	cv::Mat byteToMat(char *arr)
	{
		cv::Mat test(480, 640, CV_32FC1, arr, cv::Mat::AUTO_STEP);
		cv::imshow("test", test);
		cv::waitKey(10);
		return test;
	}

	void fileArr(std::string filename)
	{
		std::ifstream fl(filename.c_str());
		fl.seekg(0, std::ios::end);
		fileSize_ = fl.tellg();
		compressedSrc_ = new char[fileSize_];
		fl.seekg(0, std::ios::beg);
		fl.read(compressedSrc_, fileSize_);
		fl.close();
	}
	char *decompress(size_t decompressedLeft, size_t compressedTotal, char *compressedSrc)
	{
		char *decompressedDest = new char[decompressedLeft];
		char * decompressedDestTemp = decompressedDest;
		size_t bytesRead = compressedTotal;
		size_t destSize;
		size_t destSizeTemp = destSize;
		size_t left = compressedTotal;
		size_t out;
		while(out != 0){
		out = LZ4F_decompress(dctx_, decompressedDestTemp, &destSizeTemp, compressedSrc, &left, NULL);
		ROS_INFO("out: %Ld, Read: %Ld, Wrote: %Ld", out, left, destSizeTemp);
		compressedSrc += left;
		read_+=left;
		decompressedDestTemp += destSizeTemp;
		left = out;
		destSizeTemp = destSize - destSizeTemp;
		}
		return decompressedDest;
	}

private:
	CompressBroadcaster broadcast_;
	std::string rgbFile_;
	std::string depthFile_;
	cv::VideoCapture cap_;
	char * compressedSrc_;
	size_t fileSize_;
	size_t read_;
	LZ4F_decompressionContext_t dctx_;
};

main(int argc, char **argv)
{
	ros::init(argc, argv, "broadcast_node");
	ros::NodeHandle nh;
	CompressBroadcaster cb(nh);
	SyncedDecompressor *sd = new SyncedDecompressor("RGBout.avi", "test", cb);
	delete sd;
	ros::spin();
	return 0;
}