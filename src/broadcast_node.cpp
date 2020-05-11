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
// broadcasts data to relevant topics
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
		// create rgb image from opencv mar
		cv::Mat decompressedRGB = rgbImage;
		cv_bridge::CvImage outRGB_msg;
		outRGB_msg.header = std_msgs::Header();
		outRGB_msg.header.frame_id = "camera_rgb_optical_frame";

		outRGB_msg.encoding = sensor_msgs::image_encodings::BGR8;
		outRGB_msg.image = decompressedRGB;

		//create depth image form opencv mat
		cv::Mat compressDepthImage = depthImage;
		cv_bridge::CvImage outDepth_msg;
		outDepth_msg.header = std_msgs::Header();
		outDepth_msg.header.frame_id = "camera_rgb_optical_frame";
		outDepth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
		outDepth_msg.image = compressDepthImage;

		// publish images with same time stamps
		ros::Time cur = ros::Time::now();
		outDepth_msg.header.stamp = cur;
		outRGB_msg.header.stamp = cur;
		rgbVideo.publish(outRGB_msg.toImageMsg());
		depthVideo.publish(outDepth_msg.toImageMsg());
	}
};
// decompresses frame by frame syncing rgb and depth data
class SyncedDecompressor
{
public:
	SyncedDecompressor(std::string rgbFile, std::string depthFile, int w, int h, int frames, CompressBroadcaster cb) : rgbFile_(rgbFile), depthFile_(depthFile), broadcast_(cb), cap_(rgbFile_), width(w), height(h), fps(frames)
	{
		// set up to decompress data
		std::ifstream *fl = fileStream(depthFile_);
		read_ = 0;
		compressedSrcOrig_ = new char[100];
		compressedSrc_ = compressedSrcOrig_;
		inputBufSize = 0;
		// check to ensure rgb video exists
		if (!cap_.isOpened())
			std::cout << "no video" << std::endl;
		// ensure loop runs at rate of of rgb video
		ros::Rate rate(cap_.get(CV_CAP_PROP_FPS));
		while (cap_.isOpened())
		{
			// get one frame from rgb video decompressed
			cv::Mat rgbFrame;
			cap_ >> rgbFrame;
			// end loop if at end of video
			if (rgbFrame.empty())
				break;
			// decompress total bytes and generate matrix from lz4 file
			cv::Mat depthFrame(height, width, CV_32FC1);
			size_t total = width * height * depthFrame.elemSize();
			LZ4F_createDecompressionContext(&dctx_, LZ4F_VERSION);
			// hand bytes to read and pointer to next frame of data
			char *decompressDest = decompress(total, fl);
			depthFrame = byteToMat(decompressDest);
			// publish the two images decompressed at same time
			broadcast_.publish(rgbFrame, depthFrame);
			// delete temp array of bytes
			delete decompressDest;
			// wait for refresh rate
			rate.sleep();
		}
		// free utlities and end
		cap_.release();
		fl->close();
		delete fl;
		delete compressedSrcOrig_;
		LZ4F_freeDecompressionContext(dctx_);
	}
	// convert byte array into matric of of type 32cf1
	cv::Mat byteToMat(char *arr)
	{
		cv::Mat test(height, width, CV_32FC1, arr, cv::Mat::AUTO_STEP);
		return test;
	}
	// gets size of file and creates file stream
	std::ifstream *fileStream(std::string filename)
	{
		std::ifstream *fl = new std::ifstream(filename.c_str());
		if (fl->fail())
			std::cout << "File not found" << std::endl;
		fl->seekg(0, std::ios::end);
		fileSize_ = fl->tellg();
		fl->seekg(0, std::ios::beg);
		return fl;
	}
	// decompresses data from lz4 format
	char *decompress(size_t decompressedLeft, std::ifstream *fl)
	{
		// create init variables
		char *decompressedDest = new char[decompressedLeft];
		char *decompressedDestTemp = decompressedDest;
		size_t bytesRead = decompressedLeft;
		size_t destSize;
		size_t destSizeTemp = destSize;
		size_t left = 15;
		size_t out = left;
		// loops until frame is decompressed out = 0, out refers to as the hint to the number of bytes left to decompress frame
		while (out != 0)
		{
			// if we need more bytes from file, reached end of array, read decompressedLeft bytes in or until end
			if (inputBufSize == 0)
			{
				delete (compressedSrcOrig_);
				compressedSrcOrig_ = new char[decompressedLeft];
				compressedSrc_ = compressedSrcOrig_;
				if (read_ + decompressedLeft <= fileSize_)
				{
					fl->read(compressedSrc_, decompressedLeft);
					inputBufSize = decompressedLeft;
				}
				else
				{
					fl->read(compressedSrc_, fileSize_ - read_);
					ROS_INFO("end");
					inputBufSize = fileSize_ - read_;
				}
			}
			// decompress at max left bytes into destTemp of size destSizeTemp
			// decompressDestTemp pointer to the array of bytes decompressed
			// destSizeTemp tells the number of bytes left in dest array, and is changed by decompress to bytes written
			// compressedSrc is the pointer to the currect byte to be decompressed from
			// left is the bytes to try and decompress in order to write destSizeTemp bytes, decompress changes this to bytes read
			out = LZ4F_decompress(dctx_, decompressedDestTemp, &destSizeTemp, compressedSrc_, &left, NULL);
			// advance pointer to next bytes after reading left bytes
			compressedSrc_ += left;
			//reduce size remaining in arr
			inputBufSize -= left;
			// take count of total bytes read in object
			read_ += left;
			// advance pointer in dest array
			decompressedDestTemp += destSizeTemp;
			// insert hint in out bytes to read next
			left = out;
			// if hint > size then next loop will read till only the end then more bytes will be fetched
			if (left > inputBufSize)
				left = inputBufSize;
			// decrease space availiable in dest array
			destSizeTemp = destSize - destSizeTemp;
		}
		// return pointer to start of array with decompressed bytes
		return decompressedDest;
	}

private:
	CompressBroadcaster broadcast_;
	std::string rgbFile_;
	std::string depthFile_;
	cv::VideoCapture cap_;
	char *compressedSrc_;
	char *compressedSrcOrig_;
	size_t fileSize_;
	size_t read_;
	LZ4F_decompressionContext_t dctx_;
	int width;
	int height;
	int fps;
	size_t inputBufSize;
};

main(int argc, char **argv)
{
	// note expects static cam_info with ros time of 0 init in launch file
	ros::init(argc, argv, "broadcast_node");
	ros::NodeHandle nh;
	// parameter handling
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
	// debug data
	ROS_INFO("RGB File: %s", rgbFile.c_str());
	ROS_INFO("Depth File: %s", depthFile.c_str());
	// create broadcaster
	CompressBroadcaster cb(nh);
	// create decompressor that syncs both depth and rgb data and sends decompressed data to broadcaster
	SyncedDecompressor *sd = new SyncedDecompressor(rgbFile.c_str(), depthFile.c_str(), width, height, fps, cb);
	// free space
	delete sd;
	ros::spin();
	return 0;
}