//this is specific to the problem; that is, pose type is suited to 3d states but our states are similar to Pose2d

#include <armadillo>
#include "geometry_msgs/PoseStamped.h"
#include <stdexcept>
#include <string>

void check_vec_valid(arma::vec);

arma::vec pose_to_vec(geometry_msgs::PoseStamped ps){
	arma::vec v(3);
	v(0) = ps.pose.position.x;
	v(1) = ps.pose.position.y;
	v(2) = 2*atan2(ps.pose.orientation.z, ps.pose.orientation.w);
	if(v(2)>M_PI) v(2)-=2*M_PI;
	if(v(2)<-M_PI) v(2)+=2*M_PI;
	return v;
}

std::vector<double> pose_to_std_vec(geometry_msgs::PoseStamped ps){
	std::vector<double> v;
	v.push_back(ps.pose.position.x);
	v.push_back(ps.pose.position.y);
	double theta = 2*atan2(ps.pose.orientation.z, ps.pose.orientation.w);
	if(theta>M_PI) theta-=2*M_PI;
	if(theta<-M_PI) theta+=2*M_PI;
	v.push_back(theta);
	return v;
}

geometry_msgs::PoseStamped vec_to_pose(arma::vec v){
	check_vec_valid(v);
	geometry_msgs::PoseStamped ps;
	ps.header.stamp = ros::Time::now();
	ps.pose.position.x = v(0);
	ps.pose.position.y = v(1);
	ps.pose.orientation.z = sin(0.5*v(2));
	ps.pose.orientation.w = cos(0.5*v(2));
	return ps;
}

void check_vec_valid(arma::vec v){
	if(v.size()!=3){
		std::string s = "Vector to be converted must be of length 3. It's size is "+std::to_string((int)v.size());
		throw std::runtime_error(s.c_str());
	}
}