#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <chrono>

// Constants - I declared here because inside the class does not work
double g = 9.81;

const int LEGS = 4;
const int NUM_STATE = 13; // 3 for position, 3 for velocity, 3 for orientation, 3 for angular velocity + 1 to add the gravity term
const int NUM_DOF = 3 * LEGS; // 1 GRF for each foot which is 3x1 vector so 3*LEGS = 12
const Eigen::Matrix<double, 3, LEGS> foot_positions = Eigen::Matrix<double, 3, LEGS>::Zero(); // 3x4 matrix

const int HORIZON_LENGTH = 10; // 10 steps
const double dt = 0.01; // 0.01 seconds

const Eigen::Matrix3d A1_INERTIA_WORLD = Eigen::Matrix3d::Identity(); // Inertia matrix of the robot in the world frame
const double ROBOT_MASS = 10; // 10 kg

void vectorToSkewSymmetric(Eigen::Vector3d vector, Eigen::Matrix3d &skew_symmetric){
    skew_symmetric << 0, -vector(2), vector(1),
                      vector(2), 0, -vector(0),
                      -vector(1), vector(0), 0;
}

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
        A_matrix_discrete = Eigen::Matrix<double, NUM_STATE, NUM_STATE>::Zero();
        B_matrix_continuous = Eigen::Matrix<double, NUM_STATE, NUM_DOF>::Zero();
        B_matrix_discrete = Eigen::Matrix<double, NUM_STATE, NUM_DOF>::Zero();
    }
    
    /**
     * @brief Sets the rotation matrix based on the given Euler angles.
     * The robot’s orientation is expressed as a vector of Z-Y-X Euler angles Θ = [φ θ ψ]ᵀ where ψ is the yaw, θ is the pitch, and φ is the roll.
     * 
     * @param euler_angles The Euler angles representing the rotation of the body respect to the inertial/world frame?
     */
    Eigen::Matrix3d setRotationMatrix(Eigen::Vector3d euler_angles){
        double psi = euler_angles(2);

        Eigen::Matrix3d Rotation_z;
        
        // Rotation matrix around the z-axis
        Rotation_z << cos(psi), -sin(psi), 0,
                      sin(psi), cos(psi), 0,
                      0, 0, 1;
        
        return Rotation_z;
    }

    void setQMatrix()
    {}

    void setRMatrix()
    {}

    
    auto setAMatrixContinuous(Eigen::Matrix3d Rotation_z)
    {
        A_matrix_continuous.block<3, 3>(0, 6) = Rotation_z;
        A_matrix_continuous.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
        // std::cout << "A_matrix_continuous: \n" << A_matrix_continuous << std::endl;

        return A_matrix_continuous;
    }

    void setAMatrixDiscrete(Eigen::Matrix<double, NUM_STATE, NUM_STATE> A_matrix_continuous)
    {
        // First order approximation of the matrix exponential
        A_matrix_discrete = Eigen::Matrix<double, NUM_STATE, NUM_STATE>::Identity(NUM_STATE, NUM_STATE) + A_matrix_continuous * dt;
        // std::cout << "A_matrix_discrete: \n" << A_matrix_discrete << std::endl;
    }

    void setBMatrixContinuous(Eigen::Matrix3d foot_positions)
    {
        for (int i=0; i<LEGS; i++)
        {
            Eigen::Vector3d r = foot_positions.col(i);
            Eigen::Matrix3d skew_symmetric_foot_position;
            vectorToSkewSymmetric(r, skew_symmetric_foot_position);
            B_matrix_continuous.block<3, 3>(6, 3*i) = A1_INERTIA_WORLD.inverse() * skew_symmetric_foot_position;
            B_matrix_continuous.block<3, 3>(9, 3*i) = Eigen::Matrix3d::Identity() * (1/ROBOT_MASS);
        }
    }

    void setBMatrixDiscrete(Eigen::Matrix<double, NUM_STATE, NUM_DOF> B_matrix_continuous)
    {
        B_matrix_discrete = B_matrix_continuous * dt;
    }

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
    Eigen::Matrix<double, NUM_STATE, NUM_STATE> A_matrix_discrete;
    Eigen::Matrix<double, NUM_STATE, NUM_DOF> B_matrix_continuous;
    Eigen::Matrix<double, NUM_STATE, NUM_DOF> B_matrix_discrete;

};

int main(){
    
    auto start = std::chrono::high_resolution_clock::now();

    MPC mpc;
    mpc.initMatricesZero();
    auto Rotation_z = mpc.setRotationMatrix(Eigen::Vector3d(0.5, 0.7, 0.6));
    mpc.setAMatrixContinuous(Rotation_z);
    mpc.setAMatrixDiscrete(mpc.A_matrix_continuous);
    mpc.setBMatrixContinuous(foot_positions);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;

    std::cout << "Time taken: " << duration.count() << " milliseconds" << std::endl;

    return 0;
}