#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include <stdlib.h>
#include <time.h>
#include <iostream>
class Robot{
	ros::Publisher pose_pub;
	ros::Subscriber control_sub;
	ros::Subscriber uppc_sub;
	double x, y, theta, t_prev;
	std::vector<double> x_init;
public:
	Robot(ros::NodeHandle nh){
		nh.getParam("x_init", x_init);

		t_prev = ros::Time::now().toSec();
		x = x_init[0]; y = x_init[1];
		theta = -3.14158/4;
		srand(getpid() * time(NULL));
		x += ((double) rand() / (RAND_MAX))*0.1;

		control_sub = nh.subscribe("/control_input", 100, &Robot::controlCallback, this);
		pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/pose", 100);
	}
	
	void controlCallback(const geometry_msgs::Twist::ConstPtr& msg){
		double t = ros::Time::now().toSec();
		double dt = t - t_prev;
		t_prev = t;
		if(dt>0.1)return;

		double c = cos(theta);
		double s = sin(theta);
		double dx = (msg->linear.x*c - msg->linear.y*s) * dt * 0.001;
		double dy = (msg->linear.x*s + msg->linear.y*c) * dt * 0.001;
		double dtheta = msg->angular.z * dt;
		x += dx;
		y += dy;
		theta += dtheta; 
		if(theta<-M_PI) theta+=2*M_PI;
		if(theta>M_PI) theta-=2*M_PI;
	}

	void publish(){
		geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "odom";
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        pose_stamped.pose.orientation.z = sin(theta*0.5);
        pose_stamped.pose.orientation.w = cos(theta*0.5);
		pose_pub.publish(pose_stamped);
	}
};

int main(int argc, char* argv[]){
	ros::init(argc, argv, "robot_node");
	ros::NodeHandle nh;
	Robot r(nh);
	
	ros::Rate rate(100);
	while(nh.ok()){
		ros::spinOnce();

		r.publish();

		rate.sleep();
	}
}
