#include "ros/ros.h"
#include "hybrid_controller/ControlInput.h"
#include "geometry_msgs/PoseStamped.h"
#include <boost/bind.hpp>
#include <string>
#include "PPC.hpp"
#include <armadillo>
#include "arma_ros_conversions.h"
//#include "exprtk.hpp"


class ControllerNode{
	ros::Publisher control_input_pub;
	std::vector<ros::Subscriber> pose_subs;
	std::vector<geometry_msgs::PoseStamped> poses;
	int n_robots, robot_id;
	std::string formula, formula_type;
	double a, b;

	PPC prescribed_performance_controller;

	bool state_was_read = false;
public:
	ControllerNode(ros::NodeHandle nh, ros::NodeHandle priv_nh){

		nh.param("n_robots", n_robots, 1);
		priv_nh.getParam("robot_id", robot_id);
		nh.getParam("formula"+std::to_string(robot_id), formula);
		nh.getParam("formula_type"+std::to_string(robot_id), formula_type);
		nh.getParam("a"+std::to_string(robot_id), a);
		nh.getParam("b"+std::to_string(robot_id), b);

		control_input_pub = nh.advertise<hybrid_controller::ControlInput>("/control_input_robot"+std::to_string(robot_id), 100);

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
		if(!state_was_read && i == robot_id){
			arma::vec x = pose_to_vec(poses[robot_id]);
			prescribed_performance_controller.init(a, b, formula_type, formula, ros::Time::now().toSec(), x);
			state_was_read = true;
		}
		//ROS_INFO("%s, %f, %d", ros::this_node::getName().c_str(), msg->pose.position.x, i );
	}

	void update(){
		if(!state_was_read) return;
		arma::vec u = prescribed_performance_controller.u(pose_to_vec(poses[robot_id]), ros::Time::now().toSec());
		hybrid_controller::ControlInput u_msg = vec_to_control_input(u);
		control_input_pub.publish(u_msg);
	}
};

int main(int argc, char* argv[]){
	ros::init(argc, argv, "hybrid_controller_node");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh("~");

	ControllerNode controller_node(nh, priv_nh);

	ros::Rate rate(100);
	while(ros::ok()){
		ros::spinOnce();
		controller_node.update();
		rate.sleep();
	}
}
