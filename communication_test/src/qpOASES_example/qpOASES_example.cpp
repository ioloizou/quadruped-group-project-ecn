#include "qpOASES.hpp"

/** Example to solve a the following QP using qpOASES:
 * min f(x) = 1/2x^T Q x + c^T x  
 * st: 
 * g1(x) = -x_1 + 2x_2 <= 1 
 * g2(x) =  3x_1 + x_2 <= 2
 * 0 <= x_1 <= 1
 * 0 <= x_2 <= 1
 * 
 * Where Q = [4 -1; -1 2], c = [1; 1]
 * 
 * We will probably have to implement a eigen_to_qpOASES_matrix function to convert Eigen matrices to qpOASES matrices
 * If we get the hessians and gradients from casadi
 * 
 * 
 * To run the code, first compile: g++ -o qpOASES_example qpOASES_example.cpp -L~/Downloads/qpOASES-stable-3.2/bin -lqpOASES
 * then run: ./qpOASES_example
 * (these commands should be run from the communication_test/src/qpOASES_example folder)
 */
int main(){

// Initialize variables: 
const int nV = 2;
const int nC = 2;

//initialize the variables (lba = nullptr)
qpOASES::real_t H[nV*nV] = {4, -1, -1, 2};
qpOASES::real_t g[nV] = {1, 1};
qpOASES::real_t A[nC*nV] = {-1,2,3,1};
qpOASES::real_t lb[nV] = {0,0};
qpOASES::real_t ub[nV] = {1, 1};
qpOASES::real_t ubA[nC]= {1, 2};

int nWSR = 10;


// Create a QP object
qpOASES::QProblem my_qp(nV, nC);

//solve
my_qp.init(H, g, A, lb, ub, nullptr, ubA, nWSR);

//get the solution
qpOASES::real_t xOpt[nV];
my_qp.getPrimalSolution(xOpt);
printf("Optimal solution: x1 = %f, x2 = %f\n", xOpt[0], xOpt[1]);



                                                     
}

