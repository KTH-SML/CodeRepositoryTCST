#ifndef PPC_HPP
#define PPC_HPP

#include <math.h>
#include <armadillo> 
#include <string>
#include <stdexcept>
#include "CriticalEventParam.h"

class PPC{
    double gamma_0, gamma_inf, l, rho_max, r, t_0, t_star;
    double zeta_u, zeta_l;
    arma::vec u_max;
    int K, k=0;

    double a, b;
    std::string formula, formula_type;
public:
    void (*criticalEventCallback)(CriticalEventParam);

    void init(
        double a, 
        double b, 
        std::string formula_type, 
        std::string formula, 
        double rho_opt, 
        double t_0, 
        int K,
        arma::vec x);

    arma::mat g(arma::vec x);
    arma::vec f(arma::vec x);
    arma::vec f_c(arma::vec X);

    double rho_psi(arma::vec x);
    arma::vec drho_psi(arma::vec x);

    double gamma(double t);

    double e(arma::vec x, double t);

    arma::vec u(arma::vec x, double t);

    bool detect(double rho_psi);

    void repair(arma::vec x, double t);

    void setUmax(arma::vec u_max);
};
    
#endif   
