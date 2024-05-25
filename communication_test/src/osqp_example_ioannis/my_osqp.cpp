#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <cmath>
#include <iostream>
#include <chrono>

// Constants - I declared here because inside the class does not work
double g = 9.81; // m/s^2

const int LEGS = 4;
const int NUM_STATE = 13; // 3 for position, 3 for velocity, 3 for orientation, 3 for angular velocity + 1 to add the gravity term
const int NUM_DOF = 3 * LEGS; // 1 GRF for each foot which is 3x1 vector so 3*LEGS = 12
const Eigen::Matrix<double, 3, LEGS> foot_positions = Eigen::Matrix<double, 3, LEGS>::Random(); // 3x4 matrix with random values until we get the real values

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
        // Parameters initialization with values form paper
        mu = 0.6;
        fz_min = 10;
        fz_max = 666;
        states = Eigen::VectorXd::Random(NUM_STATE); // Dummy values until we get the real states
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
        Aqp_matrix = Eigen::Matrix<double, NUM_STATE * HORIZON_LENGTH, NUM_STATE>::Zero();
        Bqp_matrix = Eigen::Matrix<double, NUM_STATE * HORIZON_LENGTH, NUM_DOF * HORIZON_LENGTH>::Zero();
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

    void setQMatrix(Eigen::VectorXd &q_weights)
    {
        // It doesnt let us declare size at the start so we need to initialize it here
        Q_matrix = Eigen::SparseMatrix<double>(NUM_STATE * HORIZON_LENGTH, NUM_STATE * HORIZON_LENGTH);
        for (int i = 0; i < NUM_STATE * HORIZON_LENGTH; i++)
        {
            // We insert weights in the diagonal of the matrix but we multiply by 2 
            // because the QP solver expects a 0.5 factor in front of the quadratic term
            Q_matrix.insert(i, i) = 2 * q_weights(i % NUM_STATE);
        }
        // std::cout << "Q_matrix: \n" << Q_matrix << std::endl;
    }

    void setRMatrix(Eigen::VectorXd &r_weights)
    {
        R_matrix = Eigen::SparseMatrix<double>(NUM_DOF * HORIZON_LENGTH, NUM_DOF * HORIZON_LENGTH);
        for (int i = 0; i < NUM_DOF * HORIZON_LENGTH; i++)
        {
            R_matrix.insert(i, i) = 2 * r_weights(i % NUM_DOF);
        }
        // std::cout << "R_matrix: \n" << R_matrix << std::endl;
    }

    
    auto setAMatrixContinuous(Eigen::Matrix3d Rotation_z)
    {
        // Using the paper A matrix as reference
        A_matrix_continuous.block<3, 3>(0, 6) = Rotation_z;
        A_matrix_continuous.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
        A_matrix_continuous(11, 12) = 1; // Because of the augmented gravity term in the state space model
        // std::cout << "A_matrix_continuous: \n" << A_matrix_continuous << std::endl;

        return A_matrix_continuous;
    }

    auto setAMatrixDiscrete(Eigen::Matrix<double, NUM_STATE, NUM_STATE> A_matrix_continuous)
    {
        // First order approximation of the matrix exponential
        A_matrix_discrete = Eigen::Matrix<double, NUM_STATE, NUM_STATE>::Identity(NUM_STATE, NUM_STATE) + A_matrix_continuous * dt;
        // std::cout << "A_matrix_discrete: \n" << A_matrix_discrete << std::endl;
        return A_matrix_discrete;
    }

    auto setBMatrixContinuous()
    {
        for (int i=0; i<LEGS; i++)
        {
            // Using the paper B matrix as reference
            Eigen::Vector3d r = foot_positions.col(i);
            Eigen::Matrix3d skew_symmetric_foot_position;
            vectorToSkewSymmetric(r, skew_symmetric_foot_position);
            B_matrix_continuous.block<3, 3>(6, 3*i) = A1_INERTIA_WORLD.inverse() * skew_symmetric_foot_position;
            B_matrix_continuous.block<3, 3>(9, 3*i) = Eigen::Matrix3d::Identity() * (1/ROBOT_MASS);
        }
        // std::cout << "B_matrix_continuous: \n" << B_matrix_continuous << std::endl;
        return B_matrix_continuous;
    }

    auto setBMatrixDiscrete(Eigen::Matrix<double, NUM_STATE, NUM_DOF> B_matrix_continuous)
    {
        B_matrix_discrete = B_matrix_continuous * dt;
        // std::cout << "B_matrix_discrete: \n" << B_matrix_discrete << std::endl;
        return B_matrix_discrete;
    }

    auto setAqpMatrix(Eigen::Matrix<double, NUM_STATE, NUM_STATE> A_matrix_discrete)
    {
        for (int i = 0; i < HORIZON_LENGTH; i++)
        {
            if (i == 0)
            {   // First block is the A_discrete matrix
                Aqp_matrix.block<NUM_STATE, NUM_STATE>(i * NUM_STATE, 0) = A_matrix_discrete;
            }
            else   
            {  // The current block is the evolution of the previous block A(k+2) = A(k+1) * A(k)
                Aqp_matrix.block<NUM_STATE, NUM_STATE>(i * NUM_STATE, 0) = Aqp_matrix.block<NUM_STATE, NUM_STATE>((i - 1) * NUM_STATE, 0) * A_matrix_discrete;
            }
        }
        // std::cout << "Aqp_matrix: \n" << Aqp_matrix << std::endl;
        return Aqp_matrix;
    }

    // Jumps from 2 milliseconds to 30. NEEDS optimization!
    void setBqpMatrix(Eigen::Matrix<double, NUM_STATE, NUM_DOF> B_matrix_discrete, Eigen::Matrix<double, NUM_STATE * HORIZON_LENGTH, NUM_STATE> Aqp_matrix)
    {
        for (int i = 0; i< HORIZON_LENGTH; i++)
        {
            for (int j=0; j<= i; j++){
                if (i - j== 0)
                {   // The diagonal blocks are the B_discrete matrix
                    Bqp_matrix.block<NUM_STATE, NUM_DOF>(i * NUM_STATE, 0) = B_matrix_discrete;
                }
                else
                {   // I am not sure about this part
                    Bqp_matrix.block<NUM_STATE, NUM_DOF>(i * NUM_STATE, j * NUM_DOF) = Aqp_matrix.block<NUM_STATE, NUM_STATE>((i -j - 1) * NUM_STATE, 0) * B_matrix_discrete;
                }
            }
        }
        std::cout << "Bqp_matrix: \n" << Bqp_matrix << std::endl;
    }

    // OSQP QP formulation
    // minimize 0.5 * x^T * P * x + q^T * x
    // subject to l <= A * x <= u
    // So every constain equality and inequality need to be converted to this form
    void setAcMatrix()
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
    Eigen::VectorXd states; // Dummy values until we get the real states

    //Matrices declaration
    Eigen::Matrix<double, NUM_STATE, NUM_STATE> A_matrix_continuous;
    Eigen::Matrix<double, NUM_STATE, NUM_STATE> A_matrix_discrete;
    Eigen::Matrix<double, NUM_STATE, NUM_DOF> B_matrix_continuous;
    Eigen::Matrix<double, NUM_STATE, NUM_DOF> B_matrix_discrete;
    Eigen::SparseMatrix<double> Q_matrix;
    Eigen::SparseMatrix<double> R_matrix;
    Eigen::Matrix<double, NUM_STATE * HORIZON_LENGTH, NUM_STATE> Aqp_matrix;
    Eigen::Matrix<double, NUM_STATE * HORIZON_LENGTH, NUM_DOF * HORIZON_LENGTH> Bqp_matrix;

    // Eigen::Matrix<double,
};

int main(){
    
    auto start = std::chrono::high_resolution_clock::now();

    MPC mpc;
    mpc.initMatricesZero();

    Eigen::VectorXd q_weights = Eigen::VectorXd::Ones(NUM_STATE);
    mpc.setQMatrix(q_weights);

    Eigen::VectorXd r_weights = Eigen::VectorXd::Ones(NUM_DOF);
    mpc.setRMatrix(r_weights);

    auto Rotation_z = mpc.setRotationMatrix(Eigen::Vector3d(0.5, 0.7, 0.6));
    mpc.setAMatrixContinuous(Rotation_z);
    mpc.setAMatrixDiscrete(mpc.A_matrix_continuous);
    mpc.setBMatrixContinuous();
    mpc.setBMatrixDiscrete(mpc.B_matrix_continuous);
    mpc.setAqpMatrix(mpc.A_matrix_discrete);
    mpc.setBqpMatrix(mpc.B_matrix_discrete, mpc.Aqp_matrix);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;

    std::cout << "Time taken: " << duration.count() << " milliseconds" << std::endl;

    return 0;
}