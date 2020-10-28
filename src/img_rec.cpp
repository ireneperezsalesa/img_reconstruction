// RECONSTRUCTION - GREY BACKGROUND, USING PREVIOUS IMAGE TO ADD NEW EVENTS

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
cv_bridge::CvImage last;

int counter = 1;
bool got_first_image = false; 

void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
	if (got_first_image == false)
	{	
		cv_image.image = cv::Mat(msg->height, msg->width, CV_8UC1);
        	cv_image.image = cv::Scalar(128);
		cv_image.encoding="mono8";
		got_first_image = true;
	}
	else {
		cv_image.image = last.image;
	}

	for (int i = 0; i < msg->events.size(); ++i)
	{
        	const int x = msg->events[i].x;
        	const int y = msg->events[i].y;

        	
            	if (msg->events[i].polarity == true && cv_image.image.at<uint8_t>(cv::Point(x, y)) < (uint8_t)225)  
		{
			cv_image.image.at<uint8_t>(cv::Point(x, y))+=(uint8_t)30; // adjust value here
		}
		else if (msg->events[i].polarity == true) 
		{
			cv_image.image.at<uint8_t>(cv::Point(x, y))=(uint8_t)255;
		}		
		else if (msg->events[i].polarity == false && cv_image.image.at<uint8_t>(cv::Point(x, y)) > (uint8_t)30)
		{
			cv_image.image.at<uint8_t>(cv::Point(x, y))-=(uint8_t)30; 
		}
		else 
		{
			cv_image.image.at<uint8_t>(cv::Point(x, y))=(uint8_t)0;
		}
		
	}

	last.image = cv_image.image;

	ros::Time last_event_time = msg->events[msg->events.size()/2].ts;
	ros::Time current_time = ros::Time::now();
	ros::Duration reconstruction_time = current_time - last_event_time;

	std::cout << "Reconstruction time: " << reconstruction_time.toSec() << std::endl;
}





int main(int argc, char **argv) 
{
	ros::init(argc, argv, "reconstructor");

	ros::NodeHandle n; 

	image_transport::ImageTransport it(n);

	// subscribe to /dvs/events 
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
