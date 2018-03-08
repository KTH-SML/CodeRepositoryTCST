#include "ros/ros.h"
#include "hybrid_controller/ControlInput.h"
#include "hybrid_controller/CriticalEvent.h"
#include "geometry_msgs/PoseStamped.h"
#include <boost/bind.hpp>
#include <string>
#include <vector>
#include "PPC.hpp"
#include <armadillo>
#include "arma_ros_conversions.h"
#include "exprtk.hpp"

class ControllerNode{
	ros::Publisher control_input_pub, critical_event_pub;
	std::vector<ros::Subscriber> pose_subs;

	std::vector<geometry_msgs::PoseStamped> poses;

	int n_robots, robot_id, K;
	std::string formula, formula_type;
	double a, b, rho_opt;
	arma::vec u_max;

	PPC prescribed_performance_controller;

	bool state_was_read = false;

public:
	ControllerNode(ros::NodeHandle nh, ros::NodeHandle priv_nh){
		readParameters(nh, priv_nh);
		prescribed_performance_controller.setUmax(u_max);

		control_input_pub = nh.advertise<hybrid_controller::ControlInput>("/control_input_robot"+std::to_string(robot_id), 100);
		critical_event_pub = nh.advertise<hybrid_controller::CriticalEvent>("/critical_event"+std::to_string(robot_id), 100);

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

	void readParameters(ros::NodeHandle nh, ros::NodeHandle priv_nh){
		nh.param("n_robots", n_robots, 1);
		priv_nh.getParam("robot_id", robot_id);
		std::string robot_id_str = std::to_string(robot_id);
		nh.getParam("formula"+robot_id_str, formula);
		nh.getParam("formula_type"+robot_id_str, formula_type);
		nh.getParam("a"+robot_id_str, a);
		nh.getParam("b"+robot_id_str, b);
		nh.getParam("rho_opt"+robot_id_str, rho_opt);
		nh.getParam("K", K);
		std::vector<double> u_max_stdvec;
		nh.getParam("u_max", u_max_stdvec);
		u_max = arma::vec(u_max_stdvec);
	}

	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int i){
		poses[i] = *msg;
		if(!state_was_read && i == robot_id){
			arma::vec x = pose_to_vec(poses[robot_id]);
			prescribed_performance_controller.init(
				a, b, formula_type, formula, rho_opt, 
				ros::Time::now().toSec(), K, x);
			state_was_read = true;
		}
	}

	void update(){
		if(!state_was_read) return;
		arma::vec u = prescribed_performance_controller.u(pose_to_vec(poses[robot_id]), ros::Time::now().toSec());

		hybrid_controller::ControlInput u_msg = vec_to_control_input(u);
		control_input_pub.publish(u_msg);
	}

	void setCriticalEventCallback(void (*callback)(CriticalEventParam)){
		prescribed_performance_controller.criticalEventCallback = callback;
	}

	void publishCriticalEvent(CriticalEventParam critical_event_param){
		hybrid_controller::CriticalEvent ce_msg;
		critical_event_pub.publish(ce_msg);
	}
};

class CriticalEvent{
public:
	static ControllerNode* controller_node;
	static void criticalEventCallback(CriticalEventParam critical_event_param){
		controller_node->publishCriticalEvent(critical_event_param);
	}
};
ControllerNode* CriticalEvent::controller_node;

int main(int argc, char* argv[]){
	ros::init(argc, argv, "hybrid_controller_node");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh("~");

	ControllerNode controller_node(nh, priv_nh);
	CriticalEvent::controller_node = &controller_node;
	controller_node.setCriticalEventCallback(CriticalEvent::criticalEventCallback);

	ros::Rate rate(100);
	while(ros::ok()){
		ros::spinOnce();
		controller_node.update();
		rate.sleep();
	}
}
