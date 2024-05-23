// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"

// eigen
#include <Eigen/Dense>

#include <iostream>

int main(){
    void setRotationMatrix();
    
    void setQMatrix();
    void setRMatrix();

    void setAMatrixContinious();
    void setAMatrixDiscrete();
    void setBMatrixContinious();
    void setBMatrixDiscrete();

    void setEqualityConstraints();
    void setInequalityConstraints();

    void setInitialGuess();
    void setBounds();

    

    void solveQP();
    void printResults();


    return 0;
}