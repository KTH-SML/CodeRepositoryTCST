#ifndef PPC_HPP
#define PPC_HPP

#include <math.h>
#include <armadillo> 
#include <string>
#include <stdexcept>

class PPC{
    double gamma_0, gamma_inf, l, rho_max, r, t_0, t_star;//should be defined in the constructor
public:
    void init(double a, double b, std::string formula_type, std::string formula, double t_0, arma::vec x);

    arma::mat g(arma::vec x);
    arma::vec f(arma::vec x);
    arma::vec f_c(arma::vec X);

    double rho_psi(arma::vec x);
    //double rho_conjunction(parsed functions);
    arma::vec drho_psi(arma::vec x);

    double gamma(double t);

    double e(arma::vec x, double t);

    arma::vec u(
        arma::vec x, 
        double t);
    };
    
#endif   
