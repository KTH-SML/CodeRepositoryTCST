struct CollaborationRequestParam{
    double r, rho_max, gamma_0, gamma_inf,  l, t_star;
    int c;

    CollaborationRequestParam(int c, double t_star, double r, double rho_max, double gamma_0, double gamma_inf, double l): 
        c(c), r(r), rho_max(rho_max), gamma_0(gamma_0), gamma_inf(gamma_inf), 
        l(l), t_star(t_star){
        }

    CollaborationRequestParam(int c): c(c){
    }
};