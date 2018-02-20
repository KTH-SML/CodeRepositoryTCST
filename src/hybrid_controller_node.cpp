#include "ros/ros.h"
#include "hybrid_controller/ControlInput.h"
#include "geometry_msgs/PoseStamped.h"
#include <boost/bind.hpp>
#include <string>
#include "PPC.hpp"


class ControllerNode{
	ros::Publisher control_input_pub;
	std::vector<ros::Subscriber> pose_subs;
	std::vector<geometry_msgs::PoseStamped> poses;
	int n_robots;
public:
	ControllerNode(ros::NodeHandle nh){
		control_input_pub = nh.advertise<hybrid_controller::ControlInput>("/control_input", 100);

		nh.param("n_robots", n_robots, 1);
		poses = std::vector<geometry_msgs::PoseStamped>(n_robots);

		std::string topic_base = "/pose_robot";
		for(int i=0; i<n_robots; i++){
			pose_subs.push_back(
				nh.subscribe<geometry_msgs::PoseStamped>(
					topic_base + std::to_string(i),
					100,
					boost::bind(&ControllerNode::poseCallback, this, _1, i)));
		}
	}

	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int i){
		poses[i] = *msg;
		ROS_INFO("%s, %f, %d", ros::this_node::getName().c_str(), msg->pose.position.x, i );
	}

	void update(){
		//compute and publish control signal
	}
};

int main(int argc, char* argv[]){
	ros::init(argc, argv, "hybrid_controller_node");
	ros::NodeHandle nh;

	ControllerNode controller_node(nh);

	PPC prescribed_performance_controller;

	ros::Rate rate(10);
	while(ros::ok()){
		ros::spinOnce();
		controller_node.update();
		rate.sleep();
	}
}
