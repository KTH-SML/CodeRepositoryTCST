#include <vector>

class PPC{
    //parameter values should be computed and store for each agent
    double gamma_0, gamma_inf, l, rho_max, r;
public:    
    std::vector<double> g(std::vector<double> x);
    std::vector<double> f(std::vector<double> x);
    std::vector<double> f_c(std::vector<double> X);
    double rho_psi(std::vector<double>* X, double t);
    double drho_psi(std::vector<double>* X, double t);
    std::vector<double> u(
        std::vector<double>* g,
        std::vector<double>* e, 
        std::vector<double>* X, 
        std::vector<double>* x, 
        double t);
};