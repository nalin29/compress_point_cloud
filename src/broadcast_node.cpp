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
	ros::NodeHandle node;
	image_transport::ImageTransport it;
	image_transport::Publisher rgbVideo;

public:
	CompressBroadcaster(): it(node){
		rgbVideo = it.advertise("/camera/rgb/image_color", 1);
	}
	void publish(cv::Mat compressImage){
		cv_bridge::CvImage out_msg;
		out_msg.header   = std_msgs::Header();
    	out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    	out_msg.image    = compressImage;
		rgbVideo.publish(out_msg.toImageMsg());
	}
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "broadcast_node");
  cv::VideoCapture cap("rgbOut.avi");
  int i = 0;
  if (!cap.isOpened()){
  	std::cout << "no video" << std::endl;
  	return -1;
  }
  CompressBroadcaster cb;
  while (1){
  	cv::Mat frame;
  	cap >> frame;
  	std::cout << i++ << std::endl;
  	if (frame.empty()){
  		break;
  	}
  	cb.publish(frame);
  }
  cap.release();
  ros::spin();
  return 0;
}