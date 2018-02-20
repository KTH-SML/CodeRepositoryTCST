#include "PPC.hpp"

#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)<(b))?(a):(b))
#define ABS(a) ((a)>0?(a):(-a))

void PPC::init(double a, double b, std::string formula_type, std::string formula, double t_0, arma::vec x){
    // first, parse the formula
    this->t_0 = t_0;
    double rho_0 = rho_psi(x);
    rho_max = MAX(0, rho_0) + 1.0; // replace constant 1.0
    r = rho_max * arma::randu() / 2;

    if(formula_type == "G"){
        t_star = a;
    }
    else if(formula_type == "F"){
        t_star = a + (b-a)/3;
    }
    else{
        throw std::runtime_error("Incorrect formula type");
    }

    if(t_star > 0.01){
        gamma_0 = (rho_max - rho_0)+ABS(rho_max - rho_0);
    }
    else{
        gamma_0 = (rho_max-rho_0) + (rho_0 - r)*arma::randu();
    }

    gamma_inf = MIN(gamma_0, rho_max - r)*arma::randu();

    if(-gamma_0 + rho_max < r){// also should be t_star>0
        l = -log((r + gamma_inf - rho_max) / (-gamma_0 + gamma_inf))/t_star;
    }
    else{//??
        l = 0;
    }
}

arma::mat PPC::g(arma::vec x){
    return arma::eye<arma::mat>(x.n_elem, x.n_elem);
}

arma::vec PPC::f(arma::vec x){
    return arma::zeros<arma::vec>(x.n_elem);
}

double PPC::rho_psi(arma::vec x){
    return 1.0 - sqrt((x(0)-10) * (x(0)-10) + (x(1)-10) * (x(1)-10));
}

arma::vec PPC::drho_psi(arma::vec x){
    arma::vec drho(x.n_elem);
    double root = sqrt((x(0)-10)*(x(0)-10)+(x(1)-10)*(x(1)-10));
    if(root - 0.0 < 1e-12){
        return arma::zeros<arma::vec>(x.n_elem);
    }
    else{
        drho(0) = -(x[0]-10)/root;
        drho(1) = -(x[1]-10)/root;
        drho(2) = 0;
        return drho;
    }
}

double PPC::gamma(double t){
    return (gamma_0 - gamma_inf)*exp(-l*t) + gamma_inf;
}

double PPC::e(arma::vec x, double t){
    double epsilon = (rho_psi(x) - rho_max) / gamma(t);
    return log(-1-1/epsilon);
}

arma::vec PPC::u(
    arma::vec x, 
    double t){
        return -e(x, t-t_0)*g(x).t()*drho_psi(x);
    }
