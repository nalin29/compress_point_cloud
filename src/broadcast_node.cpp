#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
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
class CompressBroadcaster
{
	image_transport::ImageTransport it;
	image_transport::Publisher rgbVideo;
	image_transport::Publisher depthVideo;

public:
	CompressBroadcaster(ros::NodeHandle nh) : it(nh)
	{
		rgbVideo = it.advertise("/camera/rgb/image_rect_color", 1);
		depthVideo = it.advertise("/camera/depth_registered/sw_registered/image_rect", 1);
	}
	void publish(cv::Mat rgbImage, cv::Mat depthImage)
	{
		cv::Mat compressImage = rgbImage;
		cv_bridge::CvImage out_msg;
		out_msg.header = std_msgs::Header();
		out_msg.header.frame_id = "camera_rgb_optical_frame";
		//out_msg.header.stamp = ros::Time::now();
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image = compressImage;

		cv::Mat compressDepthImage = depthImage;
		cv_bridge::CvImage outDepth_msg;
		outDepth_msg.header = std_msgs::Header();
		outDepth_msg.header.frame_id = "camera_rgb_optical_frame";
		//outDepth_msg.header.stamp = ros::Time::now();
		outDepth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
		outDepth_msg.image = compressDepthImage;
		rgbVideo.publish(out_msg.toImageMsg());
		depthVideo.publish(outDepth_msg.toImageMsg());
	}
};
class SyncedDecompressor
{
public:
	SyncedDecompressor(std::string rgbFile, std::string depthFile, int w, int h, int frames, CompressBroadcaster cb) : rgbFile_(rgbFile), depthFile_(depthFile), broadcast_(cb), cap_(rgbFile_), width(w), height(h), fps(frames)
	{
		fileArr(depthFile_);
		read_ = 0;
		if (!cap_.isOpened())
			std::cout << "no video" << std::endl;
		// runs at rate of 30hz
		ros::Rate rate(cap_.get(CV_CAP_PROP_FPS));
		while (cap_.isOpened())
		{
			cv::Mat frame;
			cap_ >> frame;
			if (frame.empty())
				break;
			cv::Mat depthFrame(height, width, CV_32FC1);
			size_t total = width * height * depthFrame.elemSize();
			LZ4F_createDecompressionContext(&dctx_, LZ4F_VERSION);
			char *decompressDest = decompress(total, fileSize_ - read_, compressedSrc_ + read_);
			depthFrame = byteToMat(decompressDest);
			broadcast_.publish(frame, depthFrame);
			delete decompressDest;
			// wait for 30hz rate
			rate.sleep();
		}
		cap_.release();
		LZ4F_freeDecompressionContext(dctx_);
	}
	~SyncedDecompressor()
	{
		delete compressedSrc_;
	}
	cv::Mat byteToMat(char *arr)
	{
		cv::Mat test(height, width, CV_32FC1, arr, cv::Mat::AUTO_STEP);
		//cv::imshow("testDecompressed", test);
		//cv::waitKey(11);
		return test;
	}

	void fileArr(std::string filename)
	{
		std::ifstream fl(filename.c_str());
		if (fl.fail())
			std::cout << "File not found" << std::endl;
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
		char *decompressedDestTemp = decompressedDest;
		size_t bytesRead = compressedTotal;
		size_t destSize;
		size_t destSizeTemp = destSize;
		size_t left = compressedTotal;
		size_t out;
		while (out != 0)
		{
			out = LZ4F_decompress(dctx_, decompressedDestTemp, &destSizeTemp, compressedSrc, &left, NULL);
			//ROS_INFO("out: %Ld, Read: %Ld, Wrote: %Ld", out, left, destSizeTemp);
			compressedSrc += left;
			read_ += left;
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
	char *compressedSrc_;
	size_t fileSize_;
	size_t read_;
	LZ4F_decompressionContext_t dctx_;
	int width;
	int height;
	int fps;
};

main(int argc, char **argv)
{
	ros::init(argc, argv, "broadcast_node");
	ros::NodeHandle nh;
	std::string rgbFile;
	std::string depthFile;
	int fps;
	int width;
	int height;
	nh.param<std::string>("/broadcast_node/rgb_file", rgbFile, "RGBout.avi");
	nh.param<std::string>("/broadcast_node/depth_file", depthFile, "depth_data");
	nh.param<int>("/broadcast_node/fps", fps, 30);
	nh.param<int>("/broadcast_node/width", width, 640);
	nh.param<int>("/broadcast_node/height", height, 480);
	ROS_INFO("RGB File: %s", rgbFile.c_str());
	ROS_INFO("Depth File: %s", depthFile.c_str());
	CompressBroadcaster cb(nh);
	SyncedDecompressor *sd = new SyncedDecompressor(rgbFile.c_str(), depthFile.c_str(), width, height, fps, cb);
	delete sd;
	ros::spin();
	return 0;
}