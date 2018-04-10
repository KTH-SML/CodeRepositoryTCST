#include "PPC.hpp"

#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)<(b))?(a):(b))
#define ABS(a) ((a)>0?(a):(-a))

PPC::PPC(int robot_id, std::vector<double> a, std::vector<double> b, 
    std::vector<std::string> formula_type, std::vector<std::string> formula, 
    std::vector<std::vector<std::string>> dformula, std::vector<double> rho_opt, 
    int K, arma::vec u_max, std::vector<int> robots_in_cluster): 
        robot_id(robot_id), a(a), b(b),
        formula_type(formula_type), formula(formula), dformula(dformula),
        rho_opt(rho_opt), K(K), u_max(u_max), drho_fp(std::vector<FormulaParser<double>>(dformula[robot_id].size())),
        robots_in_cluster(robots_in_cluster){
            for(int i=0; i<robots_in_cluster.size(); i++){
                c[robots_in_cluster[i]] = (int)RobotTask::Own;
            }
    }

void PPC::init(double t_0, arma::vec x, arma::vec X){

    this->X_std = arma::conv_to<std::vector<double>>::from(X);
    this->t_0 = t_0;

    setFormulaParsers(robot_id);

    double rho_0 = rho_psi(X_std);
    rho_max = MAX(0, rho_0) + (rho_opt[robot_id] - MAX(0, rho_0))*0.9;
    zeta_u = (rho_opt[robot_id] - rho_max)/K; 
    r = rho_max * arma::randu() / 2;

    if(formula_type[robot_id] == "G"){
        t_star = a[robot_id];
    }
    else if(formula_type[robot_id] == "F"){
        t_star = a[robot_id] + (b[robot_id]-a[robot_id])/3;
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

    if(-gamma_0 + rho_max < r){
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

double PPC::rho_psi(std::vector<double> X){
    return rho_fp.value(X);
}

arma::vec PPC::drho_psi(std::vector<double> X){
    arma::vec drho(dformula[robot_id].size());
    for(int i=0; i<dformula[robot_id].size(); i++){
        drho(i) = drho_fp[i].value(X);
    }
    if(!drho.is_finite()){
        drho.zeros();
    }
    return drho;
}

double PPC::gamma(double t){
    return (gamma_0 - gamma_inf)*exp(-l*t) + gamma_inf;
}

double PPC::e(std::vector<double> X, double t){
    double epsilon = (rho_psi(X) - rho_max) / gamma(t);
    if(detect(epsilon)){
        if(k++ < K){ // stage 1
            CriticalEventParam ce;
            ce.r[0] = r; ce.rho_max[0] = rho_max; ce.gamma_0[0] = gamma_0; ce.gamma_inf[0] = gamma_inf; ce.l[0] = l; ce.t_star[0] = t_star;
            repair(X, t);
            ce.r[1] = r; ce.rho_max[1] = rho_max; ce.gamma_0[1] = gamma_0; ce.gamma_inf[1] = gamma_inf; ce.l[1] = l; ce.t_star[1] = t_star;
            criticalEventCallback(ce);
        }
        else{
            if(detectStageTwo()){ // stage 2
                c[robot_id] = robot_id;
                requestCollaboration(CollaborationRequestParam(c[robot_id], t_star, r, rho_max, gamma_0, gamma_inf, l));
            }
            else{ //stage 3
            }
        }

        if(epsilon>=0) epsilon = 0;
        if(epsilon<=-1) epsilon = -1;
    }
    return log(-1-1/epsilon);
}

arma::vec PPC::u(std::vector<double> X, std::vector<double>x, double t){
    
    if(c[robot_id] == (int)RobotTask::Own){
        if(formulaSatisfied(X, t-t_0)){
            c[robot_id] = (int)RobotTask::Free;
            formula_satisfied = true;
        }
    }
    else if(c[robot_id] == robot_id){
        if(formulaSatisfied(X, t-t_0)){
            c[robot_id] = (int)RobotTask::Free;
            formula_satisfied = true;
            requestCollaboration(CollaborationRequestParam((int)RobotTask::Free));
        }
    }
    if(c[robot_id] == (int)RobotTask::Free){
        return arma::zeros<arma::vec>(3);
    }

    double e_ = e(X, t-t_0);
    arma::vec u_;
    if(std::isinf(e_)){
        e_ = e_>0 ? u_max(0) : -u_max(0);
    }
    u_ = -e_*g(x).t()*drho_psi(X);

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

void PPC::repair(std::vector<double> X, double t){

    double rho = rho_psi(X);

    if(formula_type[robot_id] == "F"){
        t_star = b[robot_id];
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

bool PPC::formulaSatisfied(std::vector<double> X, double t){
    double rho = rho_psi(X);
    if(formula_type[robot_id] == "F" && t >= a[robot_id] && t <= b[robot_id]  
    || formula_type[robot_id] == "G" && t >= b[robot_id]){
        if( r < rho && rho < rho_max)
            return true;
    }
    return false;
}

void PPC::setCollaborationParameters(double t_star, double r, double rho_max, double gamma_0, double gamma_inf, double l){
    this->t_star = t_star;
    this->r = r;
    this->rho_max = rho_max;
    this->gamma_0 = gamma_0;
    this->gamma_inf = gamma_inf;
    this->l = l;
}

void PPC::externalCollaborationRequest(int i, int c_i, double t_star, double r, double rho_max, double gamma_0, double gamma_inf, double l){
    c[i] = c_i;
    if(c_i == i){ // collaborative control incoming request
        c[robot_id] = c_i;
        setFormulaParsers(c_i);
        setCollaborationParameters(t_star, r, rho_max, gamma_0, gamma_inf, l);
    }
    else if (c_i == (int)RobotTask::Free){
        if(formula_satisfied){
            c[robot_id] = (int)RobotTask::Free;
        }
        else{
            c[robot_id] = (int)RobotTask::Own;
            setFormulaParsers(robot_id);
            //reinitialize for own task and current time
        }
    }
}

bool PPC::detectStageTwo(){
    if(std::all_of(robots_in_cluster.begin(), robots_in_cluster.end(),
            [&](int r){
                return c[r]==(int)RobotTask::Free || c[r]==(int)RobotTask::Own && formula_type[r]=="F"?b[r]:a[r] > b[robot_id];
            })){
        return true;
    }
    return false;
}

void PPC::setFormulaParsers(int c){
    rho_fp = FormulaParser<double>(formula[c], "x", X_std);
    for(int i=0; i<dformula[c].size(); i++){
       drho_fp[i] = FormulaParser<double>(dformula[c][i], "x", X_std);
    }
}