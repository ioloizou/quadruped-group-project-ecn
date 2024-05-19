#include <IpTNLP.hpp>

using namespace Ipopt;

class IpoptExampleNLP : public TNLP
{
    public:
    
    // Constructor
    IpoptExampleNLP(); 

    // Method to request bounds on the variables and constraints
    virtual bool get_bounds_info( Index n, Number* x_l, Number* x_u, Index m, Number* g_l, Number* g_u) = 0;

    // Method to request the starting point for the optimization
    virtual bool get_starting_point( Index n, bool init_x, Number* x, bool init_z, Number* z_L, Number* z_U, Index m, bool init_lambda, Number* lambda) = 0;

    // Method to request the value of the objective function
    virtual bool eval_f( Index n, const Number* x, bool new_x, Number& obj_value) = 0;

    // Method to request the gradient of the objective function
    virtual bool eval_grad_f( Index n, const Number* x, bool new_x, Number* grad_f) = 0;

    // Method to request the value of the constraints
    virtual bool eval_g( Index n, const Number* x, bool new_x, Index m, Number* g) = 0;

    // Method to request the Jacobian of the constraints
    virtual bool eval_jac_g( Index n, const Number* x, bool new_x, Index m, Index nele_jac, Index* iRow, Index *jCol, Number* values) = 0;

    // Method to request the Hessian of the Lagrangian
    virtual bool eval_h( Index n, const Number* x, bool new_x, Number obj_factor, Index m, const Number* lambda, bool new_lambda, Index nele_hess, Index* iRow, Index* jCol, Number* values) = 0;

    // This method is called when the algorithm has finished
    virtual void finalize_solution( SolverReturn status, Index n, const Number* x, const Number* z_L, const Number* z_U, Index m, const Number* g, const Number* lambda, Number obj_value, const IpoptData* ip_data, IpoptCalculatedQuantities* ip_cq) = 0;
};