//============================================================================
// Name        : scan.cpp
// Author      : Mason Sage
// Version     :
// Description : Assignment 3 for ros curriculum- publishes location of pillar with respect to robot. Data gathered from lidar under scan.cpp
//============================================================================

//Assistance on linking Eclipse to ROS here https://www.youtube.com/watch?v=c5wEGx881qg

#include "ros/ros.h"
#include <iostream>    			 // std::cout
#include <limits>      			 // std::numeric_limits
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include <math.h>

float angleOfPillar;
float distanceToPillar;
float maxScanRange;

/**
 * Updates angle of closest object to robot
 *	Assume lidar is centered with respect to robot
 *	msg.angle_min = -2.356190 rads
 *	msg.angle_max = 2.356190 rads
 */
void pillarCallback(const sensor_msgs::LaserScan& msg){

	//extract smallest, average and angle from center
	maxScanRange = msg.range_max;
	float smallest = msg.ranges[0];// = std::numeric_limits<float>::max();
	double average = 0;
	int smallestScanIndex = -1;
	angleOfPillar = 0;
	distanceToPillar = 0;

	for(int i = 1; i< msg.ranges.size();i++){
		if(msg.ranges[i]<smallest){
			smallest = msg.ranges[i];
			smallestScanIndex = i;
		}
		average += msg.ranges[i];
	}
	average = average/msg.ranges.size();

	//Update angle and distance of pillar
	if(smallestScanIndex >-1){
		angleOfPillar = smallestScanIndex*msg.angle_increment + msg.angle_min;
		distanceToPillar = smallest;
	}

	//output refreshed values

	//ROS_INFO("%s", ss.str());
	ROS_INFO_STREAM("Values Updated! \nSmallest Lidar Value: "<< smallest
			<< "\n  Smallest Lidar Value: " << smallest
			<< "\n  Average Lidar Value: " << average
			<< "\n  Scan Index: " << smallestScanIndex
			<< "\n  Angle of pillar: "<< angleOfPillar
			);


	//ROS_INFO("Values Updated! \nSmallest Lidar Value: %f \nAverage Lidar Value: %f \nScan Index: %d \nAngle of pillar: %f",smallest, average smallestScanIndex, angleOfPillar);
}

/**
 * Main method-initializes node, and publishes topic to control velocity of husky robot
 */

int main(int argc, char **argv){

	//Initialize and assign node name
	ros::init(argc, argv, "pillarLocationPublisher");

	ros::NodeHandle nh;

	//create subscriber object
	ros::Subscriber sub = nh.subscribe("scan",1000, pillarCallback);  //subscribes to topic "scan'

	//Create publisher objects (husky velocity controller and RVIZ marker)
	ros::Publisher velocityPub = nh.advertise<geometry_msgs::Twist>("husky_velocity_controller/cmd_vel", 1000);
	ros::Publisher marker_pub  = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	//Dlare publication rate for node
	ros::Rate loop_rate(10);

	float pVal_rotate = 0.5;
	float pVal_forward = 0.5;
	float forwardVelocity = 15.5;

	nh.setParam("husky_pillar_lover_Pval_rotate",pVal_rotate); //proportional value to rotate
	nh.setParam("husky_pillar_lover_Pval_forward",pVal_forward); //proportional value to drive forward

	uint32_t shape = visualization_msgs::Marker::CYLINDER;

	visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.ns = "pillarLocationPublisher";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.250;
    marker.scale.y = 0.250;
    marker.scale.z = 2.50;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.20;
    marker.lifetime = ros::Duration();


	while(ros::ok()){

		//RVIZ Marker
	    marker.header.stamp = ros::Time::now();

	    //Husky velocity controller
		pVal_rotate  = 0; //init to be overwritten
		pVal_forward = 0; //init to be overwritten

		nh.getParam("husky_pillar_lover_Pval_rotate", pVal_rotate); //Overwrites pVal_rotate
		nh.getParam("husky_pillar_lover_Pval_forward", pVal_forward); //Overwrites pVal_forward

		geometry_msgs::Twist geom_msg;

		geom_msg.angular.z = -angleOfPillar*pVal_rotate;
		geom_msg.linear.x = (distanceToPillar/maxScanRange)*pVal_forward;

	    marker.pose.position.x = cos(angleOfPillar)*distanceToPillar;
	    marker.pose.position.y = sin(angleOfPillar)*distanceToPillar;

		velocityPub.publish(geom_msg);
		marker_pub.publish(marker);

	    ros::spinOnce();
	    loop_rate.sleep();
	}

	ros::spin();

	return 0;

}
