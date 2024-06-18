#include "ioannis_nlp.hpp"

#include <iostream>
#include <cassert>

// Constructor
using namespace Ipopt;

IpoptExampleNLP::IpoptExampleNLP(){
}

// Method to request the initial information about the problem
bool get_nlp_info( Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag, TNLP::IndexStyleEnum& index_style){
    
    // Number of variables
    n = 4;
    
    // Number of constraints, one equality constraint and one inequality constraint
    m = 2;
    
    // Number of non-zero elements in the Jacobian is 8 and it is dense
    nnz_jac_g = 8;
    
    // Number of non-zero elements in the Hessian is 16 and it is dense but because of 
    // the structure of the Hessian, we only need to return the lower left triangle
    nnz_h_lag = 10;
    
    // Use the C style indexing (0-based)
    index_style = TNLP::C_STYLE;
    
    return true;
}

// Method to request bounds on the variables and constraints
bool get_bounds_info( Index n, Number* x_l, Number* x_u, Index m, Number* g_l, Number* g_u){

    // Check the number of variables and constraints
    assert(n == 4);
    assert(m == 2);

    // Set the bounds for the variables
    for(Index i = 0; i < n; i++){
        x_l[i] = 1.0;
        x_u[i] = 5.0;
    }
    
    // The bounds for the inequality constraint
    g_l[0] = 25;
    g_u[0] = 2e19; // This is infinity
    
    // The bounds for the equality constraint
    g_l[1] = g_u[1] = 40;
    
    return true;
}

// Method to request the starting point for the optimization
bool get_starting_point( Index n, bool init_x, Number* x, bool init_z, Number* z_L, Number* z_U, Index m, bool init_lambda, Number* lambda){
    
    // We have only x starting values 
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);
    
    // Initialize the starting point
    x[0] = 1.0;
    x[1] = 5.0;
    x[2] = 5.0;
    x[3] = 1.0;
    
    return true;
}

// Method to request the value of the objective function
bool eval_f( Index n, const Number* x, bool new_x, Number& obj_value){
    
    // Check the number of variables
    assert(n == 4);
    
    // Compute the objective function
    obj_value = x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2];
    
    return true;
}

// Method to request the gradient of the objective function
bool eval_grad_f( Index n, const Number* x, bool new_x, Number* grad_f){
    
    // Check the number of variables
    assert(n == 4);
    
    // Compute the gradient of the objective function
    grad_f[0] = x[0] * x[3] + x[3] * (x[0] + x[1] + x[2]);
    grad_f[1] = x[0] * x[3];
    grad_f[2] = x[0] * x[3] + 1;
    grad_f[3] = x[0] * (x[0] + x[1] + x[2]);
    
    return true;
}

// Method to request the value of the constraints
bool eval_g( Index n, const Number* x, bool new_x, Index m, Number* g){
    
    // Check the number of variables and constraints
    assert(n == 4);
    assert(m == 2);
    
    // Compute the constraints
    g[0] = x[0] * x[1] * x[2] * x[3];
    g[1] = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3];
    
    return true;
}

// Method to request the Jacobian of the constraints
bool eval_jac_g( Index n, const Number* x, bool new_x, Index m, Index nele_jac, Index* iRow, Index *jCol, Number* values){
    
    // Check the number of variables and constraints
    assert(n == 4);
    assert(m == 2);
    
    if(values == NULL){
        // Return the structure of the Jacobian
        
        // First constraint
        iRow[0] = 0; jCol[0] = 0;
        iRow[1] = 0; jCol[1] = 1;
        iRow[2] = 0; jCol[2] = 2;
        iRow[3] = 0; jCol[3] = 3;
        
        // Second constraint
        iRow[4] = 1; jCol[4] = 0;
        iRow[5] = 1; jCol[5] = 1;
        iRow[6] = 1; jCol[6] = 2;
        iRow[7] = 1; jCol[7] = 3;
    }
    else{
        // Return the values of the Jacobian
        
        // First constraint
        values[0] = x[1] * x[2] * x[3]; // 0,0
        values[1] = x[0] * x[2] * x[3]; // 0,1
        values[2] = x[0] * x[1] * x[3]; // 0,2
        values[3] = x[0] * x[1] * x[2]; // 0,3
        
        // Second constraint
        values[4] = 2*x[0]; // 1,0
        values[5] = 2*x[1]; // 1,1
        values[6] = 2*x[2]; // 1,2
        values[7] = 2*x[3]; // 1,3
    }
    
    return true;
}

// Method to request the Hessian of the Lagrangian
bool eval_h( Index n, const Number* x, bool new_x, Number obj_factor, Index m, const Number* lambda, bool new_lambda, Index nele_hess, Index* iRow, Index* jCol, Number* values){
    
    // Check the number of variables and constraints
    assert(n == 4);
    assert(m == 2);
 
    if( values == NULL )
    {
        // return the structure. This is a symmetric matrix, fill the lower left
        // triangle only.

        // the hessian for this problem is actually dense
        Index idx = 0;
        for( Index row = 0; row < 4; row++ )
        {
            for( Index col = 0; col <= row; col++ )
            {
            iRow[idx] = row;
            jCol[idx] = col;
            idx++;
            }
        }

        assert(idx == nele_hess);
    }
    else
    {
        // return the values. This is a symmetric matrix, fill the lower left
        // triangle only

        // fill the objective portion
        values[0] = obj_factor * (2 * x[3]); // 0,0

        values[1] = obj_factor * (x[3]);     // 1,0
        values[2] = 0.;                      // 1,1

        values[3] = obj_factor * (x[3]);     // 2,0
        values[4] = 0.;                      // 2,1
        values[5] = 0.;                      // 2,2

        values[6] = obj_factor * (2 * x[0] + x[1] + x[2]); // 3,0
        values[7] = obj_factor * (x[0]);                   // 3,1
        values[8] = obj_factor * (x[0]);                   // 3,2
        values[9] = 0.;                                    // 3,3

        // add the portion for the first constraint
        values[1] += lambda[0] * (x[2] * x[3]); // 1,0

        values[3] += lambda[0] * (x[1] * x[3]); // 2,0
        values[4] += lambda[0] * (x[0] * x[3]); // 2,1

        values[6] += lambda[0] * (x[1] * x[2]); // 3,0
        values[7] += lambda[0] * (x[0] * x[2]); // 3,1
        values[8] += lambda[0] * (x[0] * x[1]); // 3,2

        // add the portion for the second constraint
        values[0] += lambda[1] * 2; // 0,0

        values[2] += lambda[1] * 2; // 1,1

        values[5] += lambda[1] * 2; // 2,2

        values[9] += lambda[1] * 2; // 3,3
    }

    return true;
}

// This method is called when the algorithm has finished
void finalize_solution( SolverReturn status, Index n, const Number* x, const Number* z_L, const Number* z_U, Index m, const Number* g, const Number* lambda, Number obj_value, const IpoptData* ip_data, IpoptCalculatedQuantities* ip_cq){
    
    // Print the solution
    std::cout << std::endl << std::endl << "Solution of the primal variables, x" << std::endl;
    for(Index i = 0; i < n; i++){
        std::cout << "x[" << i << "] = " << x[i] << std::endl;
    }
    
    std::cout << std::endl << std::endl << "Solution of the bound multipliers, z_L and z_U" << std::endl;
    for(Index i = 0; i < n; i++){
        std::cout << "z_L[" << i << "] = " << z_L[i] << std::endl;
    }
    for(Index i = 0; i < n; i++){
        std::cout << "z_U[" << i << "] = " << z_U[i] << std::endl;
    }
    
    std::cout << std::endl << std::endl << "Objective value" << std::endl;
    std::cout << "f(x*) = " << obj_value << std::endl;
    
    std::cout << std::endl << "Final value of the constraints" << std::endl;
    for(Index i = 0; i < m; i++){
        std::cout << "g(" << i << ") = " << g[i] << std::endl;
    }
    
    std::cout << std::endl << "Solution of the dual variables, lambda" << std::endl;
    for(Index i = 0; i < m; i++){
        std::cout << "lambda[" << i << "] = " << lambda[i] << std::endl;
    }
}