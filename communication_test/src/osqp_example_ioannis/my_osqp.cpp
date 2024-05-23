#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>

#include <cmath>
#include <iostream>

// Constants - I declared here because inside the class does not work
double g = 9.81;
const int NUM_STATE = 12; // 3 for position, 3 for velocity, 3 for orientation, 3 for angular velocity
const int NUM_INPUT = 4; // 4 for the control inputs 1 GRF for each foot
const int HORIZON_LENGTH = 10; // 10 steps

class MPC{
public:
    
    // Constructor
    MPC(){
        mu = 0.8;
        fz_min = 0.1;
        fz_max = 1000;
    }
   
    /**
     * Initializes the A_matrix_continuous and A_matrix_discrete matrices to zero.
     * 
     * This function sets the A_matrix_continuous and A_matrix_discrete matrices to zero.
     * A_matrix_continuous is an Eigen::Matrix<double, NUM_STATE, NUM_STATE> matrix,
     * and A_matrix_discrete is an Eigen::Matrix3d matrix.
     */
    void initMatricesZero(){
        A_matrix_continuous = Eigen::Matrix<double, NUM_STATE, NUM_STATE>::Zero();
        A_matrix_discrete = Eigen::Matrix3d::Zero();
    }
    
    /**
     * @brief Sets the rotation matrix based on the given Euler angles.
     * The robot’s orientation is expressed as a vector of Z-Y-X Euler angles Θ = [φ θ ψ]ᵀ where ψ is the yaw, θ is the pitch, and φ is the roll.
     * 
     * @param euler_angles The Euler angles representing the rotation of the body respect to the inertial frame?
     */
    Eigen::Matrix3d setRotationMatrix(Eigen::Vector3d euler_angles){
        double psi = euler_angles(2);

        Eigen::Matrix3d Rotation_z;
        // Rotation matrix
        Rotation_z << cos(psi), -sin(psi), 0,
             sin(psi), cos(psi), 0,
             0, 0, 1;
        
        return Rotation_z;
    }

    void setQMatrix()
    {}

    void setRMatrix()
    {}

    void setAMatrixContinious(Eigen::Matrix3d Rotation_z)
    {
        A_matrix_continuous.block<3, 3>(0, 6) = Rotation_z;
        A_matrix_continuous.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
        std::cout << "A_matrix_continuous: \n" << A_matrix_continuous << std::endl;
    }

    void setAMatrixDiscrete()
    {}

    void setBMatrixContinious()
    {}

    void setBMatrixDiscrete()
    {}

    void setEqualityConstraints()
    {}

    void setInequalityConstraints()
    {}

    void setBounds()
    {}

    void setGradient()
    {}

    void setHessian()
    {}

    void setInitialGuess()
    {}

    void solveQP()
    {}

    void printResults()
    {}

    // Parameters
    double mu;
    double fz_min;
    double fz_max;

    //Matrices declaration
    Eigen::Matrix<double, NUM_STATE, NUM_STATE> A_matrix_continuous;
    Eigen::Matrix3d A_matrix_discrete;

};

int main(){

    MPC mpc;
    mpc.initMatricesZero();
    auto Rotation_z = mpc.setRotationMatrix(Eigen::Vector3d(0.5, 0.7, 0.6));
    mpc.setAMatrixContinious(Rotation_z);

    return 0;
}