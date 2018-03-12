#include "ros/ros.h"
#include <string>
#include <vector>
#include <algorithm>
#include <armadillo>
#include "arma_ros_std_conversions.h"
#include "formula_parser.hpp"
#include "exprtk.hpp"
#include <iostream>
#include <typeinfo>

int n_robots, robot_id, K;
std::vector<std::string> formula, formula_type;
std::vector<std::vector<std::string>> dformula;
std::vector<int> cluster;
std::vector<double> a, b, rho_opt;
arma::vec u_max;

exprtk::expression<double> rho_expression;
std::vector<exprtk::expression<double>> drho_expression;

void readParameters(ros::NodeHandle nh, ros::NodeHandle priv_nh){
		nh.param("n_robots", n_robots, 1);
		priv_nh.getParam("robot_id", robot_id);

		formula = std::vector<std::string>(n_robots);
		formula_type = std::vector<std::string>(n_robots);
		cluster = std::vector<int>(n_robots);
		a = std::vector<double>(n_robots);
		b = std::vector<double>(n_robots);
		rho_opt = std::vector<double>(n_robots);

		for(int i=0; i<n_robots; i++){
			std::string i_str = std::to_string(i);
			nh.getParam("formula"+i_str, formula[i]);
			nh.getParam("formula_type"+i_str, formula_type[i]);
			nh.getParam("cluster"+i_str, cluster[i]);
			
			std::vector<std::string> df;
			nh.getParam("dformula"+i_str, df);
			dformula.push_back(df);

			nh.getParam("a"+i_str, a[i]);
			nh.getParam("b"+i_str, b[i]);
			nh.getParam("rho_opt"+i_str, rho_opt);
		}

		nh.getParam("K", K);

		std::vector<double> u_max_stdvec;
		nh.getParam("u_max", u_max_stdvec);
		u_max = arma::vec(u_max_stdvec);
	}

class CX{
public:
	arma::vec X;
	std::vector<double> X_vec;
};

int main(int argc, char* argv[]){
	ros::init(argc, argv, "test");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh("~");

	readParameters(nh, priv_nh);
	std::cout<<formula[robot_id]<<std::endl;
	CX c;
	c.X = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	c.X_vec = arma::conv_to<std::vector<double>>::from(c.X);
	
	FormulaParser<double> rho_fp(formula[robot_id], "x", c.X_vec);
	
	std::vector<FormulaParser<double>> drho_fp(3);
	for(int i=0; i<dformula[robot_id].size(); i++){
		std::cout<<i<<"/"<<dformula[robot_id].size()<<": "<<dformula[robot_id][i]<<std::endl;
		drho_fp[i] = FormulaParser<double>(dformula[robot_id][i], "x", c.X_vec);
		std::cout<<"address of first fp: "<<&drho_fp[0]<<std::endl;
		std::cout<<"capacity "<<drho_fp.capacity()<<std::endl;
	}

	std::cout<<"Parsers created"<<std::endl;

	c.X_vec = {0.0175058, 0.0175058, 0.0, 0.0, 0.0, 0.0};
	for(int i=0; i<drho_fp.size(); i++){
		std::cout<<i<<"/"<<dformula[robot_id].size()<<": "<<drho_fp[i].value(c.X_vec)<<std::endl;
	}
	return 0;

	int freq;
	nh.param<int>("control_freq", freq, 100);
	ros::Rate rate(freq);
	while(ros::ok()){
		ros::spinOnce();
		c.X(0) += 0.1;
		std::cout<<"X"<<c.X.t()<<std::endl;
		c.X_vec = arma::conv_to<std::vector<double>>::from(c.X);
		
		for(int i=0; i<c.X_vec.size(); i++){
			std::cout<<"\t"<<c.X_vec[i];
		}
		std::cout<<std::endl;
		std::cout<<rho_fp.value(c.X_vec)<<std::endl;

		for(int i=0; i<drho_fp.size(); i++){
			std::cout<<drho_fp[i].value(c.X_vec)<<"\t";
		}
		std::cout<<std::endl;
		rate.sleep();
	}
}

