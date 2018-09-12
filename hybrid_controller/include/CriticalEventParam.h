#include <vector>

struct CriticalEventParam{
    std::vector<double> r, rho_max, gamma_0, gamma_inf,  l, t_star;

    CriticalEventParam(): r(std::vector<double>(2)), rho_max(std::vector<double>(2)), 
                    gamma_0(std::vector<double>(2)), gamma_inf(std::vector<double>(2)),  
                    l(std::vector<double>(2)), t_star(std::vector<double>(2)){
                    }
};