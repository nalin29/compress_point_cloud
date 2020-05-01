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

class CompressBroadcaster {
	image_transport::ImageTransport it;
	image_transport::Publisher rgbVideo;
	
public:
	CompressBroadcaster(ros::NodeHandle nh): it(nh){
		rgbVideo = it.advertise("/camera/rgb/image_color", 1);
	}
	void publish(cv::Mat image){
		cv::Mat compressImage = image;
		cv_bridge::CvImage out_msg;
		out_msg.header   = std_msgs::Header();
    	out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    	out_msg.image    = compressImage;
		rgbVideo.publish(out_msg.toImageMsg());
	}
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "broadcast_node");
  ros::NodeHandle nh;
  cv::VideoCapture cap("RGBout.avi");
  CompressBroadcaster cb(nh);
  size_t i = 0;
  if (!cap.isOpened()){
  	std::cout << "no video" << std::endl;
  	return -1;
  }
  // runs at rate of 30hz
  ros::Rate rate(30.0);
  while (cap.isOpened()){
  	cv::Mat frame;
  	cap >> frame;
  	std::cout << i++ << std::endl;
  	if (frame.empty()){
  		break;
  	}
  	cb.publish(frame);
	// wait for 30hz rate
	rate.sleep();
  }
  cap.release();
  ros::spin();
  return 0;
}