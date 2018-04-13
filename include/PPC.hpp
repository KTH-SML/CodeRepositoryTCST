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
    std::vector<double> rho_opt;
    arma::vec u_max;
    int K, k=0;

    std::vector<double> a, b;
    std::vector<std::string> formula_type;
    std::vector<std::string> formula;
    std::vector<std::vector<std::string>> dformula;

    std::vector<int> V;
    std::map<int, int> c;

    FormulaParser<double> rho_fp;
    std::vector<FormulaParser<double>> drho_fp;

    bool formula_satisfied = false;

public:
    void (*criticalEventCallback)(CriticalEventParam);
    void (*requestCollaboration)(CollaborationRequestParam);
    void (*sendc)(int);

    PPC(int robot_id,
        std::vector<double> a, 
        std::vector<double> b, 
        std::vector<std::string> formula_type, 
        std::vector<std::string> formula, 
        std::vector<std::vector<std::string>> dformula,
        std::vector<double> rho_opt,
        int K,
        arma::vec u_max,
        std::vector<int> V);

    void init(double t_0, double t_r, arma::vec x, arma::vec X);

    void setCollaborationParameters(double, double, double, double, double, double); 

    void externalCollaborationRequest(arma::vec X, double t, int i, int c_i, double t_star, double r, double rho_max, double gamma_0, double gamma_inf, double l);

    arma::vec u(std::vector<double> X, std::vector<double> x, double t);

    void setc(int i, int c_i);

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

    bool formulaSatisfied(std::vector<double> X, double t);

    bool detectStageTwo();

    void setFormulaParsers(int c);
};
    
#endif   
