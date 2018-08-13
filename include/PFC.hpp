#include <armadillo>

class PFC{
    int robot_id;
    double r, R, k, w;
    arma::vec u_max;
public:
    PFC(int robot_id, double r, double R, arma::vec u_max, double w);
    arma::vec u(arma::vec X);
};