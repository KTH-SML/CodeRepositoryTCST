#include "PPC.hpp"

std::vector<double> PPC::g(std::vector<double> x){
    std::vector<double> g;
    int n = x->size();
    g.assign(n, 1.0);
    return g;
}

std::vector<double> PPC::f(std::vector<double> x){
    std::vector<double> f;
    int n = x->size();
    f.assign(n, 0.0);
    return f;
}

double PPC::rho_psi(std::vector<double> X, double t){

}