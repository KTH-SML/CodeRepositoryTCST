#ifndef PPC_HPP
#define PPC_HPP

#include <math.h>
#include <armadillo> 
#include <string>
#include <stdexcept>
#include "CriticalEventParam.h"
#include "formula_parser.hpp"

class PPC{
    std::vector<double> X_std;

    int robot_id;

    double gamma_0, gamma_inf, l, rho_max, r, t_0, t_star;
    double zeta_u, zeta_l;
    double rho_opt;
    arma::vec u_max;
    int K, k=0;

    double a, b;
    std::string formula, formula_type;
    std::vector<std::string> dformula;

    FormulaParser<double> rho_fp;
    std::vector<FormulaParser<double>> drho_fp;

public:
    void (*criticalEventCallback)(CriticalEventParam);

    PPC(
        int robot_id,
        double a, 
        double b, 
        std::string formula_type, 
        std::string formula, 
        std::vector<std::string> dformula,
        double rho_opt,
        int K,
        arma::vec u_max);

    void init(double t_0, arma::vec x, arma::vec X);

    arma::mat g(arma::vec x);
    arma::vec f(arma::vec x);
    arma::vec f_c(arma::vec X);

    double rho_psi(std::vector<double> X);
    arma::vec drho_psi(std::vector<double> X);

    double gamma(double t);

    double e(std::vector<double> X, double t);

    arma::vec u(std::vector<double> X, std::vector<double> x, double t);

    bool detect(double rho_psi);

    void repair(std::vector<double> X, double t);
};
    
#endif   
