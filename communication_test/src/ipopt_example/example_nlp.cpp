#include "example_nlp.hpp"
#include <cassert>
#include <iostream>


using namespace Ipopt;
//constructor
HS071_NLP::HS071_NLP(){}

bool get_nlp_info(
    Index&          n,
    Index&          m,
    Index&          nnz_jac_g,
    Index&          nnz_h_lag, 
    TNLP::IndexStyleEnum& index_style   //explicitly quantify the type, because vscode was complaining without it
   )
   {
        n = 4;
        m = 2;
        nnz_jac_g = 8;
        nnz_h_lag = 10;
        index_style = TNLP::C_STYLE;
        return true;
   }  

bool get_bounds_info(
      Index   n,
      Number* x_l,
      Number* x_u,
      Index   m,
      Number* g_l,
      Number* g_u
   )
   {
    assert(n == 4);
    assert(m == 2);
    
    for(Index i = 0; i < n; i++){
        //variables have an upper-bound of 5.0 and a lower bound of 5.0 (1<= x1,x2,x3,x4 <= 5)
        x_l[i] = 1.0;
        x_u[i] = 5.0;
    }

    //First constraint (g1(x) = x1*x2*x3*x4 >= 25) has a lowerbound of 25 and no upper bound (set higher than nlp_upper_inf)
    g_l[0] = 25.0;
    g_u[0] = 2e19;
    //Second constraint is equality (x1^2 + x2^2 + x3^2 + x4^2 = 40) upper and lower bounds are set to the same value.
    g_l[1] = 40.0;
    g_u[1] = 40.0; 
   }

   bool get_starting_point(
      Index   n,
      bool    init_x,
      Number* x,
      bool    init_z,
      Number* z_L,
      Number* z_U,
      Index   m,
      bool    init_lambda,
      Number* lambda
   )
   {
    //In this problem, only have initial values for x
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    //initialize x to the given starting point x0 = (1,5,5,1)
    x[0] = 1.0;
    x[1] = 5.0;
    x[2] = 5.0;
    x[3] = 1.0;

    return true;
   }

   bool eval_f(
      Index         n,
      const Number* x,
      bool          new_x,
      Number&       obj_value
   )
   {
    assert(n == 4);
    
    //in this example we ignore the new_x flag, and simply calculate the objective function
    obj_value = x[0]*x[3]*(x[0] + x[1] + x[2]) + x[2];
    
    return true;
   }

   bool eval_grad_f(
      Index         n,
      const Number* x,
      bool          new_x,
      Number*       grad_f
   )
   {
    assert(n == 4);

    //again, we ignore the new_x flag
    grad_f[0] = x[0] * x[3] + x[3] * (x[0] + x[1] + x[2]);
    grad_f[1] = x[0] * x[3];
    grad_f[2] = x[0] * x[3] + 1;
    grad_f[3] = x[0] * (x[0] + x[1] + x[2]);

    return true;
   }

   bool eval_g(
      Index         n,
      const Number* x,
      bool          new_x,
      Index         m,
      Number*       g
   )
   {
    assert(n == 4);
    assert(m == 2);

    //Ignore new_x flag and compute value of the constraint functions
    g[0] = x[0]*x[1]*x[2]*x[3];
    g[1] = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3];

    return true;
   }

   bool eval_jac_g(
      Index         n,
      const Number* x,
      bool          new_x,
      Index         m,
      Index         nele_jac,
      Index*        iRow,
      Index*        jCol,
      Number*       values
   )
   {
    assert(n == 4);
    assert(m == 2);

        if(values == NULL){
            // return the structure of the Jacobian
            // this particular Jacobian is dense
            iRow[0] = 0;
            jCol[0] = 0;
            iRow[1] = 0;
            jCol[1] = 1;
            iRow[2] = 0;
            jCol[2] = 2;
            iRow[3] = 0;
            jCol[3] = 3;
            iRow[4] = 1;
            jCol[4] = 0;
            iRow[5] = 1;
            jCol[5] = 1;
            iRow[6] = 1;
            jCol[6] = 2;
            iRow[7] = 1;
            jCol[7] = 3;
            }else{
                // return the values of the Jacobian of the constraints
        
            values[0] = x[1] * x[2] * x[3]; // 0,0
            values[1] = x[0] * x[2] * x[3]; // 0,1
            values[2] = x[0] * x[1] * x[3]; // 0,2
            values[3] = x[0] * x[1] * x[2]; // 0,3
        
            values[4] = 2 * x[0]; // 1,0
            values[5] = 2 * x[1]; // 1,1
            values[6] = 2 * x[2]; // 1,2
            values[7] = 2 * x[3]; // 1,3
        }

        return true;
    }

    bool eval_h(
      Index         n,
      const Number* x,
      bool          new_x,
      Number        obj_factor,
      Index         m,
      const Number* lambda,
      bool          new_lambda,
      Index         nele_hess,
      Index*        iRow,
      Index*        jCol,
      Number*       values
   )
    {
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

    void finalize_solution(
      SolverReturn               status,
      Index                      n,
      const Number*              x,
      const Number*              z_L,
      const Number*              z_U,
      Index                      m,
      const Number*              g,
      const Number*              lambda,
      Number                     obj_value,
      const IpoptData*           ip_data,
      IpoptCalculatedQuantities* ip_cq
   )
   {
        // here is where we would store the solution to variables, or write to a file, etc
        // so we could use the solution.
        
        // For this example, we write the solution to the console (change to some ros implementation to test out communication)
        std::cout << std::endl << std::endl << "Solution of the primal variables, x" << std::endl;
        for( Index i = 0; i < n; i++ )
        {
            std::cout << "x[" << i << "] = " << x[i] << std::endl;
        }
        
        std::cout << std::endl << std::endl << "Solution of the bound multipliers, z_L and z_U" << std::endl;
        for( Index i = 0; i < n; i++ )
        {
            std::cout << "z_L[" << i << "] = " << z_L[i] << std::endl;
        }
        for( Index i = 0; i < n; i++ )
        {
            std::cout << "z_U[" << i << "] = " << z_U[i] << std::endl;
        }
        
        std::cout << std::endl << std::endl << "Objective value" << std::endl;
        std::cout << "f(x*) = " << obj_value << std::endl;
        
        std::cout << std::endl << "Final value of the constraints:" << std::endl;
        for( Index i = 0; i < m; i++ )
        {
            std::cout << "g(" << i << ") = " << g[i] << std::endl;
        }   
   }