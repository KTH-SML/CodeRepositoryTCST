#include <armadillo>

class PFC{
    int robot_id;
    double robot_radius;
public:
    PFC(int robot_id, double robot_radius);
    arma::vec u(arma::vec X);
};