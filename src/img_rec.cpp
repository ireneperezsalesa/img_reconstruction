// RECONSTRUCTION USING FIRST APS FRAME

#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>


cv::Mat frame = cv::Mat::zeros(cv::Size(240,180), CV_8UC1);
cv_bridge::CvImage cv_image;

int counter = 1;
bool got_first_image = false; // to take just the first frame from image_raw 


void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	if (got_first_image == false)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	
		// store the cv::Mat image in "frame"
		frame = cv_ptr->image;
		
		got_first_image = true;

		std::cout << "Got first frame: " << got_first_image << std::endl; 
	}
}

void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
	ROS_INFO("I got here.");
	for (int i = 0; i < msg->events.size(); ++i)
	{
        	const int x = msg->events[i].x;
        	const int y = msg->events[i].y;

        	
            	if (msg->events[i].polarity == true && frame.at<uint8_t>(cv::Point(x, y)) < (uint8_t)225)  
		{
			frame.at<uint8_t>(cv::Point(x, y))+=(uint8_t)30; // how much should the pixel value change for each event?
		}
		else if (msg->events[i].polarity == true) 
		{
			frame.at<uint8_t>(cv::Point(x, y))=(uint8_t)255;
		}		
		else if (msg->events[i].polarity == false && frame.at<uint8_t>(cv::Point(x, y)) > (uint8_t)30)
		{
			frame.at<uint8_t>(cv::Point(x, y))-=(uint8_t)30; 
		}
		else 
		{
			frame.at<uint8_t>(cv::Point(x, y))=(uint8_t)0;
		}
		
	}
	// transform from cv::Mat with cv_bridge
	std_msgs::Header header; // empty header
	header.seq = counter; // user defined counter
	header.stamp = ros::Time::now(); // time
	cv_image = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, frame);

	counter++;
	ROS_INFO("Added events.");
}





int main(int argc, char **argv) 
{
	ros::init(argc, argv, "reconstructor");

	ros::NodeHandle n; 

	image_transport::ImageTransport it(n);

	// subscribe to /dvs/image_raw
	image_transport::Subscriber image_sub = it.subscribe("/dvs/image_raw", 1, imageCallback);
	std::cout << "Subscribed to /dvs/image_raw." << std::endl;
	ros::spinOnce();

	// subscribe to /dvs/events and do callbacks (modify frame according to event polarity in the callback)
	ros::Subscriber events_sub = n.subscribe("/dvs/events", 1, eventsCallback);
	std::cout << "Subscribed to /dvs/events." << std::endl;

	// publish new image
	image_transport::Publisher img_pub = it.advertise("reconstructed", 1);
	std::cout << "Publishing /reconstructed." << std::endl;
	
	ros::spinOnce(); // execute callbacks

	while(ros::ok()){

		// publish reconstructed image
		img_pub.publish(cv_image.toImageMsg());	
		ros::spinOnce(); 
	}

}
