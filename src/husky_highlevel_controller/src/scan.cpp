//============================================================================
// Name        : scan.cpp
// Author      : Mason Sage
// Version     :
// Description : Assignment 2 for ros curriculum
//============================================================================

//Assistance on linking Eclipse to ROS here https://www.youtube.com/watch?v=c5wEGx881qg

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>     // std::cout
#include <limits>       // std::numeric_limits

void scanCallback(const sensor_msgs::LaserScan& msg){

	float smallest = msg.ranges[0];// = std::numeric_limits<float>::max();
	double average = 0;

	for(int i = 1; i< msg.ranges.size();i++){
		if(msg.ranges[i]<smallest){
			smallest = msg.ranges[i];
		}
		average +=msg.ranges[i];
	}
	average =average/msg.ranges.size();




	ROS_INFO("Smallest Lidar Value: %f \nAverage Lidar Value: %f",smallest,average);

}

int main(int argc, char **argv){

	ros::init(argc, argv, "scanner");

	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("scan",1000,scanCallback);

	ros::spin();

	return 0;

}
