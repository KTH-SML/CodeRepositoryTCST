#include "PFC.hpp"

PFC::PFC(int robot_id, double r, double R, arma::vec u_max, double w):
    robot_id(robot_id), r(r), R(R), u_max(u_max), k(u_max[0]/(R-r)*R*r*r*r), w(w){
}

arma::vec PFC::u(arma::vec X){
    arma::vec u = arma::zeros<arma::vec>(3);
    arma::vec x = X(arma::span(3*robot_id, 3*robot_id+1));
    for(int i=0; i<X.size()/3; i++){
        if(i == robot_id) continue;
        arma::vec xo = X(arma::span(3*i, 3*i+1));
        double d = arma::as_scalar(arma::sqrt((x-xo).t()*(x-xo)));
        if(d < R){
            if(d < r){
                u(arma::span(0,1)) += w * u_max[0] * (x - xo) / d;
            }
            else{
                u(arma::span(0,1)) += k * (1/d - 1/R) / (d*d) * (x - xo) / d;
            }
        }
    }
    double norm = arma::norm(u);
    if(norm > w * u_max[0]){
        u *= w * u_max[0]/norm;
    }
    return u;
}