#ifndef EXAMPLE_HPP
#define EXAMPLE_HPP

#include "IpTNLP.hpp"       //Include the base class from IPOPT

using namespace Ipopt;

//Class for the problem to be solved, inheriting off of Ipopt::TNLP
class HS071_NLP : public Ipopt::TNLP{
    public:
    
    HS071_NLP();

   /**@name Overloaded from TNLP */
   //@{
   
   /** Method to return some info about the NLP 
    * @param[out] n number of variables
    * @param[out] m number of inequality constraints
    * @param[out] nnz_jac_g number of non zero entries in the constraint jacobian
    * @param[out] nnz_h_lag number of non zero entries in the hessian of the lagrangian
    * @param[out] inxex_style indexing style for row/col entries in the sparse matrix format
    * 
    * @return true
   */
   virtual bool get_nlp_info(
      Index&          n,
      Index&          m,
      Index&          nnz_jac_g,
      Index&          nnz_h_lag,
      IndexStyleEnum& index_style
   );

   /** Method to return the bounds of my problem 
    * @param[in] n number of variables
    * @param[in] m number of constraints
    * 
    * @param[out] x_l lowerbounds for variables x
    * @param[out] x_u upperbounds for variables x
    * @param[out] g_l lowerbounds for the constraints
    * @param[out] g_u upperbounds for the constraints
    * 
    * @return true if success, false otherwise
   */
   virtual bool get_bounds_info(
      Index   n,
      Number* x_l,
      Number* x_u,
      Index   m,
      Number* g_l,
      Number* g_u
   );

   /** Method to return the starting point for the algorithm 
    * @param[in] n number of variables (will have the same value as specified in get_nlp_info())
    * @param[in] m number of constraints (will have the same value as specified in get_nlp_info())
    * @param[in] init_x if true, this method must provide an initial value for x
    * @param[in] init_z if true, this method must provide an initial value for z
    * @param[in] init_lamda if true, this method must provide an initial value for the constraint multipliers, lambda
    * 
    * @param[out] x the initial values for the variables
    * @param[out] lambda the initial values for the constraint multipliers
    * @param[out] z_L the initial values for the upper bound multipliers
    * @param[out] z_U the initia values for the lower bound muiltipliers
    * 
    * @return true if success, false otherwise
   */
   virtual bool get_starting_point(
      Index   n,
      bool    init_x,
      Number* x,
      bool    init_z,
      Number* z_L,
      Number* z_U,
      Index   m,
      bool    init_lambda,
      Number* lambda
   );

   /** Method to return the objective value 
    * @param[in] n number of variables (will have the same value as specified in get_nlp_info())
    * @param[in] x values for the primal variables x, at which the function is to be evaluated
    * @param[in] new_x false if any evaluation method was called with the same values of x, true otherwise
    * 
    * @param[out] obj_value value of the objective function
    * 
    * @return true if success, false otherwise
   */
   virtual bool eval_f(
      Index         n,
      const Number* x,
      bool          new_x,
      Number&       obj_value
   );

   /** Method to return the gradient of the objective 
    * @param[in] n number of variables (will have the same value as specified in get_nlp_info())
    * @param[in] x values for the primal variables x, at which the function is to be evaluated
    * @param[in] new_x false if any evaluation method was called with the same values of x, true otherwise
    * 
    * @param[out] grad_f array with the values of the gradient vector of the objective function
    * 
    * @return true if success, false otherwise
   */
   virtual bool eval_grad_f(
      Index         n,
      const Number* x,
      bool          new_x,
      Number*       grad_f
   );

   /** Method to return the constraint residuals 
    * @param[in] n number of variables (will have the same value as specified in get_nlp_info())
    * @param[in] x values for the primal variables x, at which the function is to be evaluated
    * @param[in] new_x false if any evaluation method was called with the same values of x, true otherwise
    * @param[in] m number of constraints (will have the same value as specified in get_nlp_info())
    * 
    * @param[out] g array containing the constraint function values g(x).
    * 
    * @return true if success, false otherwise
   */
   virtual bool eval_g(
      Index         n,
      const Number* x,
      bool          new_x,
      Index         m,
      Number*       g
   );

   /** Method to return:
    *   1) The structure of the jacobian (if "values" is NULL)
    *   2) The values of the jacobian (if "values" is not NULL)
    * @param[in] n number of variables (will have the same value as specified in get_nlp_info())
    * @param[in] x values for the primal variables x, at which the function is to be evaluated
    * @param[in] new_x false if any evaluation method was called with the same values of x, true otherwise
    * @param[in] m number of constraints (will have the same value as specified in get_nlp_info())
    * @param[in] nele_jac number of non-zero elements in the jacobian (will have the same value as specified in get_nlp_info())
    * 
    * @param[out] iRow first call: array of length nele_jac to store the row indices of entries in the Jacobian of the constraints; 
    * later calls: NULL
    * @param[out] jCol first call: array of length nele_jac to store the column indices of entries in the Jacobian of the constraints; 
    * later calls: NULL
    * @param[out] values first call: NULL; 
    * later calls: array of length nele_jac to store the values of the entries in the Jacobian of the constraints
    * 
    * @return true if success, false otherwise 
    */
   virtual bool eval_jac_g(
      Index         n,
      const Number* x,
      bool          new_x,
      Index         m,
      Index         nele_jac,
      Index*        iRow,
      Index*        jCol,
      Number*       values
   );

   /** Method to return:
    *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
    *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
    * @param[in] n number of variables (will have the same value as specified in get_nlp_info())
    * @param[in] x values for the primal variables x, at which the function is to be evaluated
    * @param[in] new_x false if any evaluation method was called with the same values of x, true otherwise
    * @param[in] obj_factor the sigma term in the hessian
    * @param[in] m number of constraints (will have the same value as specified in get_nlp_info())
    * @param[out] lambda the values for the constraint multipliers at the point the function is being evaluated.
    * @param[in] new_lambda false if any evaluation method was called with the same values of lambda, true otherwise
    * @param[in] nele_hess number of non-zero elements in the hessian (will have the same value as specified in get_nlp_info()
    * 
    * @param[out] iRow first call: array of length nele_hess to store the row indices of entries in the Hessian of the lagrangian; 
    * later calls: NULL
    * @param[out] jCol first call: array of length nele_hess to store the column indices of entries in the Hessian of the lagrangian; 
    * later calls: NULL
    * @param[out] values first call: NULL; 
    * later calls: array of length nele_jac to store the values of the entries in the Hessian of the Lagrangian
    * 
    * @return true if success, false otherwise
    */
   virtual bool eval_h(
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
   );

   /** This method is called when the algorithm is complete so the TNLP can store/write the solution 
    * @param[in] status The status of the algorithm termination. Possible values:
      * - SUCCESS: Algorithm terminated successfully at a locally optimal point, satisfying the convergence tolerances (can be specified by options).
      * - MAXITER_EXCEEDED: Maximum number of iterations exceeded (can be specified by an option).
      * - CPUTIME_EXCEEDED: Maximum number of CPU seconds exceeded (can be specified by an option).
      * - STOP_AT_TINY_STEP: Algorithm proceeds with very little progress.
      * - STOP_AT_ACCEPTABLE_POINT: Algorithm stopped at a point that was converged, not to "desired" tolerances, but to "acceptable" tolerances (see the acceptable-... options).
      * - LOCAL_INFEASIBILITY: Algorithm converged to a point of local infeasibility. Problem may be infeasible.
      * - USER_REQUESTED_STOP: The user call-back function TNLP::intermediate_callback returned false, i.e., the user code requested a premature termination of the optimization.
      * - DIVERGING_ITERATES: It seems that the iterates diverge.
      * - RESTORATION_FAILURE: Restoration phase failed, algorithm doesn't know how to proceed.
      * - ERROR_IN_STEP_COMPUTATION: An unrecoverable error occurred while Ipopt tried to compute the search direction.
      * - INVALID_NUMBER_DETECTED: Algorithm received an invalid number (such as NaN or Inf) from the NLP; see also option check_derivatives_for_nan_inf.
      * - INTERNAL_ERROR: An unknown internal error occurred.
      * @param[in] n number of variables (will have the same value as specified in get_nlp_info())
      * @param[in] x values for the primal variables x, at which the function is to be evaluated
      * @param[in] z_L Final values of the lower bound multipliers.
      * @param[in] z_U Final values of the upper bound multipliers.
      * @param[in] m Number of constraints.
      * @param[in] g Final values of the constraint residuals.
      * @param[in] lambda Final values of the constraint multipliers.
      * @param[in] obj_value Final value of the objective function.
      * 
      * For expert users:
      * @param[in] ip_data Pointer to IpoptData (internal data).
      * @param[in] ip_cq Pointer to IpoptCalculatedQuantities (calculated quantities).
   */
   virtual void finalize_solution(
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
   );
   //@}
};

#endif // EXAMPLE_HPP