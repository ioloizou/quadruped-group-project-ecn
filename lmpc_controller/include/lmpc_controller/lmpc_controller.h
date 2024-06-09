#ifndef LMPC_CONTROLLER_H
#define LMPC_CONTROLLER_H

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <cmath>
#include <iostream>
#include <chrono>

#include <ros/ros.h>
#include <quad_utils/ros_utils.h>
#include <quad_msgs/RobotState.h>

//Constants - Declared here because inside the class does not work

//g = acceleration of gravity [m/s^2]
const double g = 9.81;

/**
 * NUM_STATE = dimension of state vector (X, Y, Z, Vx, Vy, Vz, θx, θy, θz wx, wy, wz, g)
 * (position, lin. vel., orientation, ang. velocity, gravity)
 * due to the extension to hold the gravity term it comes to dimension 13
 * 
 * foot_positions = 3x4 matrix to hold the xyz coords of each foot. (in the body frame?)
 * 
 * NUM_DOF = Each foot has 1 GRF (which is a 3x1 vector), therefore NUM_DOF = 3*LEGS = 12
*/
const int LEGS = 4;
const int NUM_STATE = 13;
const int NUM_DOF = 3 * LEGS;
/**
 * HORIZON_LENGTH = number of steps in the horizon
 * ROBOT_MASS = Mass of the robot (10 [kg])
*/
const int HORIZON_LENGTH = 10;
const double dt = 0.03;

/**
 * NUM_BOUNDS = Number of bounds for the constraints. In our case 5 since we had to divide each inequality
 *                                                    into 2 bounds (4) + the contact constraint.
 *                                                   (Decide if there is a way to generalize this or user inputs a constant)
*/
const int NUM_BOUNDS = 5;

//A1_INERTIA_BODY = Inertia Matrix of the robot in the body frame [kg*m/s^2].
//ROBOT_MASS = Mass of the robot (5.75 [kg])
const Eigen::Matrix3d A1_INERTIA_BODY = (Eigen::Matrix3d() << 
                                           0.0158533, -3.66 * std::pow(10, -5), -6.11 * std::pow(10, -5), 
                                           -3.66 * std::pow(10, -5), 0.0377999, -2.75 * std::pow(10, -5),
                                           -6.11 * std::pow(10, -5), -2.75 * std::pow(10, -5), 0.0456542).finished();
const double ROBOT_MASS = 6;


/**
 * @brief This function converts a 3D vector into a skew-symmetric matrix
 * 
 * @param[in] = vector - 3D vector to be converted.
 * @param[in] = skew_symmetric - The skew symmetric matrix to be populated by the vector components
 * 
 * @param[out] = skew_symmetric - The updated skew symmetric matrix.
 * 
 * @returns = None
*/
void vectorToSkewSymmetric(Eigen::Vector3d vector, Eigen::Matrix3d &skew_symmetric){
    skew_symmetric << 0, -vector(2), vector(1),
                      vector(2), 0, -vector(0),
                      -vector(1), vector(0), 0;
}

class MPC{
public:
    
    // Constructor
    MPC(ros::NodeHandle &nh, int type){
        
        //ROS Stuff
        nh_ = nh;
        robot_id_ = type;
        // Parameters initialization with values from paper
        mu = 0.6;
        fz_min = 10;
        fz_max = 666;        
        //Initialize variables to zero
        states = Eigen::VectorXd::Zero(NUM_STATE);
        states(13) = g;
        states_reference = Eigen::VectorXd::Zero(NUM_STATE);
        states_reference(13) = g;
        U_vector = Eigen::VectorXd::Zero(NUM_DOF*HORIZON_LENGTH);
        //foot_positions = Eigen::Matrix<double, 3, LEGS>::Zero();
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
     * @brief This function calculates the average yaw angle (psi) in the whole horizon.
     * 
     * @param ref_body_plan The reference states of the body (each row is a state vector at a given horizon step)
     * @return double The average yaw angle in the whole horizon (psi)
     */
    double extractPsi(Eigen::MatrixXd ref_body_plan){
        double yaw_sum = 0;
        for (int i = 0; i < HORIZON_LENGTH; i++)
        {
            yaw_sum += ref_body_plan(i, 5);
        }
        double psi = yaw_sum / HORIZON_LENGTH;
        return psi;
    }
    
    /**
     * @brief Sets the rotation matrix (from body to world frame) based on the given Euler angles.
     * The robot’s orientation is expressed as a vector of Z-Y-X Euler angles Θ = [φ θ ψ]ᵀ where ψ is the yaw, θ is the pitch, and φ is the roll.
     * 
     * @param euler_angles The Euler angles representing the rotation of the body respect to the inertial/world frame.
     */
    Eigen::Matrix3d setRotationMatrix(double average_yaw){
        double psi = average_yaw;

        Eigen::Matrix3d Rotation_z;
        
        // Rotation matrix around the z-axis
        Rotation_z << cos(psi), -sin(psi), 0,
                      sin(psi), cos(psi), 0,
                      0, 0, 1;
        
        return Rotation_z;
    }

    /**
     * @brief This function initializes a diagonal matrix for weights on the state error
     * It populates Q_matrix with q_weights in its diagonal entries.
     */
    void setQMatrix(){

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

    /**
     * @brief This function initializes a diagonal matrix for weights on the control inputs
     * It populates R_matrix with r_weights in its diagonal entries.
     * 
     * @param[in] = r_weights. A vector of weights to be populated in the diagonal of R_matrix
     * @param[out] = R_matrix. The R_matrix for the cost function (diagonal)
     * 
     * @returns = None
    */    
    void setRMatrix(){
        R_matrix = Eigen::SparseMatrix<double>(NUM_DOF * HORIZON_LENGTH, NUM_DOF * HORIZON_LENGTH);
        for (int i = 0; i < NUM_DOF * HORIZON_LENGTH; i++)
        {
            R_matrix.insert(i, i) = 2 * r_weights(i % NUM_DOF);
        }
        // std::cout << "R_matrix: \n" << R_matrix << std::endl;
    }

    /**
     * @brief This function initializes the A matrix of the state space in the continuous time domain
     * The matrix is detailed in the repport
     * 
     * @param[in] = Rotation_z - The rotation matrix from body to world frame, around the Z axis.
     * @param[out] = The A matrix in continuous time domain for the state space.
     * 
     * @returns = None
    */
    void setAMatrixContinuous(Eigen::Matrix3d Rotation_z){
        // Using the paper A matrix as reference
        A_matrix_continuous.block<3, 3>(0, 6) = Rotation_z;
        A_matrix_continuous.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
        A_matrix_continuous(11, 12) = 1; // Because of the augmented gravity term in the state space model
        // std::cout << "A_matrix_continuous: \n" << A_matrix_continuous << std::endl;

    }

    /**
     * @brief This function initializes the A matrix of the state space in the discrete time domain
     * The discretization is done by first order approximation of the matrix exponential: 
     * A_discrete = I + A*dt
     * 
     * @param[in] = A_matrix_continuous (The matrix in continuous time domain)
     * @param[in] = dt (the timestep)
     * 
     * @param[out] = A_matrix_discrete (The discretized A_matrix of the state space representation)
     * 
     * @returns = None
    */
    void setAMatrixDiscrete(){
        A_matrix_discrete = Eigen::Matrix<double, NUM_STATE, NUM_STATE>::Identity(NUM_STATE, NUM_STATE) + A_matrix_continuous * dt;
        // std::cout << "A_matrix_discrete: \n" << A_matrix_discrete << std::endl;
        
    }

    /**
     * TODO: Finish documenting this function with parameter description
     * @brief This function sets the B matrix of the state space representation, in the continuous time domain.
     * The matrix is detailed in the repport.
     * 
     * @param[in] = foot_positions - 
     * @param[in] = A1_INERTIA_WORLD - Inertia matrix of the robot in the world frame
     * @param[in] = ROBOT_MASS - Scalar holding the mass of the robot
     * 
     * @param[out] = B_matrix_continuous - the B matrix in the continuous time domain
     * 
     * @returns = None
    */
    void setBMatrixContinuous(Eigen::MatrixXd foot_positions, Eigen::Matrix3d Rotation_z){
        // std::cout << "foot positions: \n" << foot_positions << std::endl;
        Eigen::Matrix3d A1_INERTIA_WORLD;
        A1_INERTIA_WORLD = Rotation_z * A1_INERTIA_BODY * Rotation_z.transpose();
        for (int i=0; i<LEGS; i++)
        {        
            // Using the paper B matrix as reference
            Eigen::Vector3d r = foot_positions.row(0);
            Eigen::Matrix3d skew_symmetric_foot_position;
            vectorToSkewSymmetric(r, skew_symmetric_foot_position);
            B_matrix_continuous.block<3, 3>(6, 3*i) = A1_INERTIA_WORLD.inverse() * skew_symmetric_foot_position;
            B_matrix_continuous.block<3, 3>(9, 3*i) = Eigen::Matrix3d::Identity() * (1/ROBOT_MASS);
        }
        // std::cout << "B_matrix_continuous: \n" << B_matrix_continuous << std::endl;
    }

    /**
     * @brief This function sets the B matrix of the state space representation, in the discrete time domain.
     * The discretization is done by scaling the B matrix by the timestep: 
     * B_discrete = B*dt
     * 
     * @param[in] = B_matrix_continuous (The matrix in continuous time domain)
     * @param[in] = dt (the timestep)
     * 
     * @param[out] = B_matrix_discrete (The discretized B_matrix of the state space representation)
     * 
     * @returns = None
    */
    void setBMatrixDiscrete(){
        B_matrix_discrete = B_matrix_continuous * dt;
        // std::cout << "B_matrix_discrete: \n" << B_matrix_discrete << std::endl;
    }

    /**
     * @brief This function sets the Aqp matrix for the QP problem.
     * In order to represent the MPC as a QP we need to satisfy the solver's default formulation
     * A_qp is detailed in the repport.
     * 
     * The first block (at position 0,0) is the A_discrete matrix
     * The subsequent blocks are the evolution of the previous blocks
     * A_qp(k+2) = A_discrete(k+1) * A_discrete(k)
     * 
     * @param[in] = A_matrix_discrete - The discrete State space matrix A
     * @param[in] = HORIZON_LENGTH - The length of the horizon
     * 
     * @param[out] = A_qp Matrix for the formulation of the problem as a QP.
     * 
     * @returns = None
    */
    void setAqpMatrix(){
        for (int i = 0; i < HORIZON_LENGTH; i++)
        {
            if (i == 0)
            {  
                Aqp_matrix.block<NUM_STATE, NUM_STATE>(i * NUM_STATE, 0) = A_matrix_discrete;
            }
            else   
            {
                Aqp_matrix.block<NUM_STATE, NUM_STATE>(i * NUM_STATE, 0) = Aqp_matrix.block<NUM_STATE, NUM_STATE>((i - 1) * NUM_STATE, 0) * A_matrix_discrete;
            }
        }
        // std::cout << "Aqp_matrix: \n" << Aqp_matrix << std::endl;
    }

    /**
     * TODO: OPTIMIZE THIS IMPLEMENTATION - Jumps from 2 to 30ms
     * @brief This function sets the Bqp matrix for the QP problem.
     * In order to represent the MPC as a QP we need to satisfy the solver's default formulation
     * B_qp is detailed in the repport.
     * 
     * The diagonal blocks are the B_discrete matrix
     * The off-diagonal blocks are the multiplication of the Aqp matrix and the B_discrete matrix
     * 
     * @param[in] = B_matrix_discrete - The discrete B matrix of the state space representation
     * @param[in] = Aqp_matrix - The matrix that represents the evolution of the state space matrix
     * 
     * @param[out] = Bqp_matrix - The B matrix for the formulation of the problem as a QP.
     * 
     * @returns = None
    */
    void setBqpMatrix(){
        for (int i = 0; i< HORIZON_LENGTH; i++)
        {
            for (int j=0; j<= i; j++){
                if (i - j== 0)
                {
                    Bqp_matrix.block<NUM_STATE, NUM_DOF>(i * NUM_STATE, 0) = B_matrix_discrete;
                }
                else
                {   // I am not sure about this part
                    Bqp_matrix.block<NUM_STATE, NUM_DOF>(i * NUM_STATE, j * NUM_DOF) = Aqp_matrix.block<NUM_STATE, NUM_STATE>((i -j - 1) * NUM_STATE, 0) * B_matrix_discrete;
                }
            }
        }
        // std::cout << "Bqp_matrix: \n" << Bqp_matrix << std::endl;
    }

    /** 
     * OSQP QP formulation
     * minimize 0.5 * x^T * P * x + q^T * x
     * subject to l <= A * x <= u
     * So every constraint equality and inequality need to be converted to this form
    */
    
    /**
     * @brief This Function builds the A matrix in OSQP Notation (Here Ac for constraints)
     * Our MPC problem has 3 Inequality constraints and 1 equality constraint, they are:
     * 
     * 1)-mu*fz <= fx <= mu*fz, which becomes (-inf <= fx-mu*fz <= 0) && (0 <= fx + mu*fz <= inf).
     * 2)-mu*fz <= fy <= mu*fz, which becomes (-inf <= fy-mu*fz <= 0) && (0 <= fy + mu*fz <= inf)
     * Here, 1) and 2) are divided into two bounds to fit OSQP default formulation.
     * 
     * Here the switch is taken care of in the bounds, so our constraint becomes fzmin*switch <= fz <= fzmax*switch
     * (Therefore we encapsulate the equality constraint in an inequality constraint that is being handled by the bounds)
     * Di*ui = 0; 
     * 3)fmin <= fz <= fmax
     * 
     * @param[in] = g_block; - Matrix that will be repeatedly added to Ac to account for the constraints we defined in each leg
     * @param[in] = Ac_Matrix; - Sparse matrix populated with g_block in its diagonal
     * 
     * @param[out] = Updated Ac_matrix
     * 
     * @returns = none
     */
    void setAcMatrix(){
        g_block.resize(NUM_BOUNDS , NUM_DOF/LEGS);
        g_block << 1,0,mu,  // fx + mu*fz
                   1,0,-mu, // fx - mu*fz
                   0,1,mu,  // fy + mu*fz
                   0,1,-mu, // fy - mu*fz
                   0,0,1;   // min max fz/contact constraint (just take into bounds of fz if foot is on ground)

        Ac_matrix = Eigen::SparseMatrix<double>(NUM_BOUNDS * LEGS * HORIZON_LENGTH, NUM_DOF * HORIZON_LENGTH);
        std::vector<Eigen::Triplet<double>> tripletList;
        
        // reserve space for the triplets (40 blocks with 9 non zero entries each)
        tripletList.reserve(9 * 40);  

        int row_offset = 0;
        int col_offset = 0;

        while (row_offset + g_block.rows() <= Ac_matrix.rows() && col_offset + g_block.cols() <= Ac_matrix.cols())
        {
            for (int i = 0; i < g_block.rows(); ++i)
            {
                for (int j = 0; j < g_block.cols(); ++j)
                {
                    tripletList.push_back(Eigen::Triplet<double>(row_offset + i, col_offset + j, g_block(i, j)));
                }
            }
            
            //Move to next insertion points, to begin inserting the next block
            row_offset += 5; 
            col_offset += 3; 
        }

        Ac_matrix.setFromTriplets(tripletList.begin(), tripletList.end());
        Ac_matrix.makeCompressed();
        
        // //std::cout << "Ac_matrix: \n" << Ac_matrix << std::endl;
        // std::cout << "Ac_matrix Shape: " << Ac_matrix.rows() << " x " << Ac_matrix.cols() << std::endl;
    }

    /**
     * @brief This Function builds the bounds for the OSQP problem
     * Fitting the OSQP default formulation given by lb <= A_c <= ub
     * 
     * @param[in] = fz_min, fz_max - Actuator limits (indirectly as maximum normal force the robot is capable to generate)
     * @param[in] = mu - Friction coefficient
     * 
     * @param[out] = lower_bounds, upper_bounds - The vectors containing the bounds for the OSQP problem
    */
    void setBounds(std::vector<std::vector<bool>> contact){
        
        // Declaring the lower and upper bounds
        Eigen::VectorXd lower_bounds(NUM_BOUNDS * LEGS);
        Eigen::VectorXd upper_bounds(NUM_BOUNDS * LEGS);
        
        lower_bounds_horizon = Eigen::VectorXd::Zero(NUM_BOUNDS * LEGS * HORIZON_LENGTH);
        upper_bounds_horizon = Eigen::VectorXd::Zero(NUM_BOUNDS * LEGS * HORIZON_LENGTH);

        int horizon_step = 0;
        
        while (horizon_step < HORIZON_LENGTH)
        {
            for (int i=0; i<LEGS; i++)
            {
            lower_bounds.segment<5>(i*NUM_BOUNDS) << 0,                                         //  0        <= fx + mu*fz
                                                     -std::numeric_limits<double>::infinity(),  // -infinity <= fx - mu*fz
                                                     0,                                         //  0        <= fy + mu*fz
                                                     -std::numeric_limits<double>::infinity(),  // -infinity <= fy - mu*fz
                                                     fz_min*contact[horizon_step][i];           //  fz_min   <= fz          fz_min*contact = 0 or 1 depending on the contact
                                                     
            upper_bounds.segment<5>(i*NUM_BOUNDS) << std::numeric_limits<double>::infinity(),   //  fx + mu*fz <= infinity
                                                     0,                                         //  fx - mu*fz <= 0
                                                     std::numeric_limits<double>::infinity(),   //  fy + mu*fz <= infinity
                                                     0,                                         //  fy - mu*fz <= 0
                                                     fz_max*contact[horizon_step][i];           //  fz <= fz_max            fz_max*contact = 0 or 1 depending on the contact
            }

            lower_bounds_horizon.segment<NUM_BOUNDS * LEGS>(horizon_step*NUM_BOUNDS*LEGS) = lower_bounds;
            upper_bounds_horizon.segment<NUM_BOUNDS * LEGS>(horizon_step*NUM_BOUNDS*LEGS) = upper_bounds;
            horizon_step += 1;
        }
        
        // std::cout<<"Lower bounds: \n"<<lower_bounds_horizon<<std::endl;
        // std::cout<<"Upper bounds: \n"<<upper_bounds_horizon<<std::endl;

        // std::cout << "lower_bounds_horizon Shape: " << lower_bounds_horizon.rows() << " x " << lower_bounds_horizon.cols() << std::endl;
        // std::cout << "upper_bounds_horizon Shape: " << upper_bounds_horizon.rows() << " x " << upper_bounds_horizon.cols() << std::endl;
    }

    /**
     * @brief This function calculates the Hessian of the cost function
     * The mathematical proof for the derivation of this expression can be seen in the repport (and in miro)
     * 
     * @param[in] = Bqp_matrix , Q_matrix, R_matrix
     * @param[out] = hessian of the cost function
     * 
     * @returns = none
    */
    void setHessian(){   
        hessian = Bqp_matrix.transpose() * Q_matrix * Bqp_matrix + R_matrix;
    
        // std::cout << "Hessian: \n" << hessian << std::endl;
        // std::cout << "Shape: " << hessian.rows() << " x " << hessian.cols() << std::endl;

    }

    /**
     * @brief This function calculates the gradient of the cost function
     * The mathematical proof for the derivation of this expression can be seen in the repport (and in miro)
     * It is vital to precompute the hessian (saves around 24ms in the overall implementation)
     * 
     * @param[in] = Bqp_matrix , Q_matrix, hessian, U_vector, states, states_reference
     * @param[out] = gradient of the cost function
     * 
     * @returns = none
    */
    void setGradient(Eigen::VectorXd grf_plan_, Eigen::VectorXd current_state_, Eigen::VectorXd ref_body_plan_){
        gradient.resize(NUM_DOF * HORIZON_LENGTH, 1);
        
        //Manipulate the inputs to satisfy our QP Implementation
        states.segment(0,12) = current_state_;
        states_reference = ref_body_plan_.row(0).transpose();
        
        //Populate the U_vector with the values in grf_plan_:
        for(int i = 0; i < HORIZON_LENGTH; i++){
            U_vector.segment(i*NUM_DOF, 1) = grf_plan_.row(i).transpose();
        }
        
        auto temp = hessian * U_vector;
        gradient = temp + 2*Bqp_matrix.transpose() * Q_matrix * (Aqp_matrix * (states - states_reference));

        // gradient = Bqp_matrix.transpose() * Q_matrix * (Aqp_matrix *states) - states_reference;


        //std::cout << "Gradient: \n" << gradient << std::endl;
        //std::cout << "Shape: " << gradient.rows() << " x " << gradient.cols() << std::endl; 
    }

    /** 
     * @brief This function sets the initial guess for the solver.
     * If the solver is running for the first time, the initial guess is a vector of zeros.
     * If it is the second or higher iterations, the initial guess is a hot-start with the previous iteration's minimizer
     * 
     * @param[in] = None
     * 
     * @param[out] = The initial guess for the solver
     * 
     * @returns = None
    */    
    void setInitialGuess(){
        Eigen::VectorXd initial_guess = Eigen::VectorXd::Zero(NUM_STATE-1);
        if (is_first_run == false){
            //Retrieve the last minimizer correctly, for now it is a random vector
            //Goal is to have: initial_guess = last_minimizer
            initial_guess.setRandom();
        }

        //std::cout << "Initial Guess: \n" << initial_guess << std::endl;
    }

    Eigen::VectorXd solveQP(){
        //Instantiate the solver
        OsqpEigen::Solver solver;

        //Configure the solver
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(false);
        solver.data()->setNumberOfVariables(NUM_DOF*HORIZON_LENGTH);
        solver.data()->setNumberOfConstraints(NUM_BOUNDS*LEGS*HORIZON_LENGTH);
        solver.data()->setLinearConstraintsMatrix(Ac_matrix);
        solver.data()->setHessianMatrix(hessian);
        solver.data()->setGradient(gradient);
        solver.data()->setLowerBound(lower_bounds_horizon);
        solver.data()->setUpperBound(upper_bounds_horizon);

        //Init and solve keeping track of time at each step
        auto t0 = std::chrono::high_resolution_clock::now();
        solver.initSolver();
        auto t1 = std::chrono::high_resolution_clock::now();
        solver.solveProblem();
        auto t2 = std::chrono::high_resolution_clock::now();

        //Process and print time intervals of the solver
        std::chrono::duration<double, std::milli> init_duration = t1 - t0;
        std::chrono::duration<double, std::milli> solve_duration = t2 - t1;
        std::cout << "Solver init time: " << init_duration.count() << "ms" << std::endl; 
        std::cout << "Solve time: " << solve_duration.count() << "ms" << std::endl;

        //print results
        Eigen::VectorXd result = solver.getSolution();
        // std::cout << "Result Shape: " << result.rows() << " x " << result.cols() << std::endl;
        std::cout << "Result: \n" << result.head(12) << std::endl;
        return result;
    }

    void printResults()
    {}


    //Update functions (to not set matrices from ground up all over again):
    // void updateMatrices(Eigen::VectorXd state){

    // }

    //ROS Stuff
    ros::NodeHandle nh_;
    int robot_id_;

    // Parameters
    double mu;
    double fz_min;
    double fz_max;

    Eigen::VectorXd states; // Dummy values until we get the real states
    Eigen::VectorXd states_reference; // Dummy values until we get the real states
    Eigen::VectorXd U_vector;
    
    
    //Matrices declaration
    Eigen::Matrix<double, NUM_STATE, NUM_STATE> A_matrix_continuous;
    Eigen::Matrix<double, NUM_STATE, NUM_STATE> A_matrix_discrete;
    Eigen::Matrix<double, NUM_STATE, NUM_DOF> B_matrix_continuous;
    Eigen::Matrix<double, NUM_STATE, NUM_DOF> B_matrix_discrete;
    Eigen::SparseMatrix<double> Q_matrix;
    Eigen::SparseMatrix<double> R_matrix;
    Eigen::Matrix<double, NUM_STATE * HORIZON_LENGTH, NUM_STATE> Aqp_matrix;
    Eigen::Matrix<double, NUM_STATE * HORIZON_LENGTH, NUM_DOF * HORIZON_LENGTH> Bqp_matrix;

    Eigen::Matrix<double,NUM_BOUNDS , NUM_DOF/LEGS> g_block;
    Eigen::SparseMatrix<double> Ac_matrix;
    
    Eigen::VectorXd lower_bounds_horizon;
    Eigen::VectorXd upper_bounds_horizon;
    
    Eigen::Matrix<double, NUM_DOF * HORIZON_LENGTH, 1> gradient;
    Eigen::SparseMatrix<double> hessian;

    //Eigen::Matrix<double, 3, LEGS> foot_positions;  


    bool is_first_run = true;  //to be set to false after first iteration, so that the initial guess is correctly set to hot-start the solver

    Eigen::VectorXd q_weights = Eigen::VectorXd::Ones(NUM_STATE);
    Eigen::VectorXd r_weights = Eigen::VectorXd::Ones(NUM_DOF);
};

#endif //LMPC_CONTROLLER_H