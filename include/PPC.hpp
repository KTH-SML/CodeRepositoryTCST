#ifndef PPC_HPP
#define PPC_HPP

#include <math.h>
#include <armadillo> 
#include <string>
#include <stdexcept>
#include <algorithm>
#include "CriticalEventParam.h"
#include "CollaborationRequestParam.h"
#include "formula_parser.hpp"

enum class RobotTask{Free = -2, Own = -1};

class PPC{
    std::vector<double> X_std;

    int robot_id;

    double gamma_0, gamma_inf, l, rho_max, r, t_0, t_star;
    double zeta_u, zeta_l;
    double rho_opt;
    arma::vec u_max;
    int K, k=0;

    std::vector<double> a, b;
    std::vector<std::string> formula_type;
    std::string formula;
    std::vector<std::string> dformula;

    std::vector<int> robots_in_cluster;
    std::map<int, int> c;

    FormulaParser<double> rho_fp;
    std::vector<FormulaParser<double>> drho_fp;

public:
    void (*criticalEventCallback)(CriticalEventParam);
    void (*collaborationRequest)(CollaborationRequestParam);

    PPC(int robot_id,
        std::vector<double> a, 
        std::vector<double> b, 
        std::vector<std::string> formula_type, 
        std::string formula, 
        std::vector<std::string> dformula,
        double rho_opt,
        int K,
        arma::vec u_max,
        std::vector<int> robots_in_cluster);

    void init(double t_0, arma::vec x, arma::vec X);

    void setCollaborationParameters(double, double, double, double, double, double); 

    void externalTaskChangeCallback(int i, int c_i, double t_star, double r, double rho_max, double gamma_0, double gamma_inf, double l);

    arma::vec u(std::vector<double> X, std::vector<double> x, double t);

private:
    arma::mat g(arma::vec x);
    arma::vec f(arma::vec x);
    arma::vec f_c(arma::vec X);

    double rho_psi(std::vector<double> X);
    arma::vec drho_psi(std::vector<double> X);

    double gamma(double t);

    double e(std::vector<double> X, double t);

    bool detect(double rho_psi);

    void repair(std::vector<double> X, double t);

    bool formula_satisfied(std::vector<double> X, double t);

    bool detect_stage_two();
};
    
#endif   
