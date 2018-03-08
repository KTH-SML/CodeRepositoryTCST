#include "PPC.hpp"

#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)<(b))?(a):(b))
#define ABS(a) ((a)>0?(a):(-a))

void PPC::init(double a, double b, std::string formula_type, std::string formula, 
    double rho_opt, double t_0, int K, arma::vec x){

    this->t_0 = t_0;
    this->K = K;
    this->a = a;
    this->b = b;
    this->formula = formula;
    this->formula_type = formula_type;

    double rho_0 = rho_psi(x);
    rho_max = MAX(0, rho_0) + 1.0; // replace constant 1.0
    zeta_u = (rho_opt - rho_max)/K;
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
    else{
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
    if(detect(epsilon)){
        if(k++ < K){
            CriticalEventParam ce; //fill critical event inforamtion before and after repair
            repair(x, t);
            criticalEventCallback(ce);
        }
        if(epsilon>=0) epsilon = 0;
        if(epsilon<=-1) epsilon = -1;
    }
    return log(-1-1/epsilon);
}

arma::vec PPC::u(arma::vec x, double t){
    double e_ = e(x, t-t_0);
    arma::vec u_;
    if(std::isinf(e_)){
        e_ = e_>0 ? u_max(0) : -u_max(0);
    }
    u_ = -e_*g(x).t()*drho_psi(x);

    double c = arma::as_scalar(u_.rows(0,1).t()*u_.rows(0,1));
    if(c > u_max(0)){ 
        u_(0) *= u_max(0)/c;
        u_(1) *= u_max(0)/c;
    }
    u_(2) = u_(2)>0 ? (u_(2)>u_max(1) ? u_max(1) : u_(2)) : (u_(2)<-u_max(1) ? -u_max(1) : u_(2));
    return u_;
}

bool PPC::detect(double epsilon){
    return !(-1<epsilon && epsilon<0);
}

void PPC::repair(arma::vec x, double t){

    double rho = rho_psi(x);

    if(formula_type == "F"){
        t_star = b;
    }
    
    rho_max += zeta_u; // rho_max must always be < rho_opt

    r *= 0.5;

    if(t_star - t > 0.01){
        zeta_l = 1.0; // change this
    }
    else{
        zeta_l = (rho - r)*arma::randu();
    }
    double gamma_tr = rho_max - rho + zeta_l;

    gamma_inf = MIN(gamma_tr, rho_max - r)*arma::randu();

    if(-gamma_tr + rho_max < r){
        l = -log((r + gamma_inf - rho_max) / (-gamma_tr + gamma_inf))/t_star;
    }
    else{
        l = 0;
    }

    gamma_0 = (gamma_tr - gamma_inf)*exp(l*t) + gamma_inf;
}

void PPC::setUmax(arma::vec u_max){
    this->u_max = u_max;
}