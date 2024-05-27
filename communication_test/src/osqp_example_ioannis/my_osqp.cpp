#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <cmath>
#include <iostream>
#include <chrono>

// Constants - I declared here because inside the class does not work
const double g = 9.81; // m/s^2

const int LEGS = 4;
const int NUM_STATE = 13; // 3 for position, 3 for velocity, 3 for orientation, 3 for angular velocity + 1 to add the gravity term
const int NUM_DOF = 3 * LEGS; // 1 GRF for each foot which is 3x1 vector so 3*LEGS = 12
const Eigen::Matrix<double, 3, LEGS> foot_positions = Eigen::Matrix<double, 3, LEGS>::Random(); // 3x4 matrix with random values until we get the real values

const int HORIZON_LENGTH = 10; // 10 steps
const double dt = 0.01; // 0.01 seconds

const int NUM_BOUNDS = 5; //4 bounds, since we divided each inequality constraint into 2 bounds + the contact equality constraint (maybe this needs to be generalized and not a fixed number later on)

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
        // Parameters initialization with values from paper
        mu = 0.6;
        fz_min = 10;
        fz_max = 666;
        states = Eigen::VectorXd::Random(NUM_STATE); // Dummy values until we get the real states
        states_reference = Eigen::VectorXd::Random(NUM_STATE); // Dummy values until we get the real states
        U_vector = Eigen::VectorXd::Random(NUM_DOF*HORIZON_LENGTH); // Dummy values until we get the real states
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
        Ac_matrix = Eigen::Matrix<double, NUM_BOUNDS * LEGS, NUM_DOF>::Zero();

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
        // std::cout << "Bqp_matrix: \n" << Bqp_matrix << std::endl;
    }

    // OSQP QP formulation
    // minimize 0.5 * x^T * P * x + q^T * x
    // subject to l <= A * x <= u
    // So every constain equality and inequality need to be converted to this form
    
    /**
    This Function builds the A matrix in OSQP Notation
    Our MPC problem has 3 Inequality constraints and 1 equality constraint, they are:
    -mu*fz <= fx <= mu*fz, which becomes (-inf <= fx-mu*fz <= 0) && (0 <= fx + mu*fz <= inf). The constraint is divided into two bounds to fit OSQP default formulation
    -mu*fz <= fy <= mu*fz, which becomes (-inf <= fy-mu*fz <= 0) && (0 <= fy + mu*fz <= inf)
    Di*ui = 0; Here the switch is taken care of in the bounds, so our constraint becomes fzmin*switch <= fz <= fzmax*switch (and this also includes the 3rd inequality constraint)
    fmin <= fz <= fmax

    *@param[in] = g_block; - Matrix that will be repeatedly added to Ac to account for the constraints we defined in each leg
    *@param[in] = Ac_Matrix; - Sparse matrix populated with g_block in its diagonal
    *@param[out] = Updated Ac_matrix
    *@returns = none
    */
    void setAcMatrix(){
        //Not sure this is the best way to do this, will leave it like that and think of a way to better generalize it:
        g_block << 1,0,mu,  // fx + mu*fz
                   1,0,-mu, // fx - mu*fz
                   0,1,mu,  // fy + mu*fz
                   0,1,-mu, // fy - mu*fz
                   0,0,1;   // min max fz/contact constraint (just take into bounds of fz if foot is on ground)

        for (int i = 0; i < LEGS; i++){ 
            Ac_matrix.block<NUM_BOUNDS, NUM_DOF/LEGS>(i*NUM_BOUNDS, i*(NUM_DOF/LEGS)) = g_block;
        }
        // Ac_matrix = Ac_matrix.sparseView();

        // std::cout << "Ac_matrix: \n" << Ac_matrix << std::endl;
    }

    /**
    This Function builds the bounds for the OSQP problem
    **/
    void setBounds()
    {
        // Declaring the lower and upper bounds
        Eigen::VectorXd lower_bounds(NUM_BOUNDS * LEGS);
        Eigen::VectorXd upper_bounds(NUM_BOUNDS * LEGS);
        
        // Setting the bounds for each leg
        for (int i=0; i<LEGS; i++)
        {
            lower_bounds.segment<5>(i*NUM_BOUNDS) << 0,                 //  0        <= fx + mu*fz
                                                     -Eigen::Infinity,  // -infinity <= fx - mu*fz
                                                     0,                 //  0        <= fy + mu*fz
                                                     -Eigen::Infinity,  // -infinity <= fy - mu*fz
                                                     fz_min;            //  fz_min   <= fz          fz_min*contact = 0 or 1 depending on the contact
                                                     
            upper_bounds.segment<5>(i*NUM_BOUNDS) << Eigen::Infinity,   //  fx + mu*fz <= infinity
                                                     0,                 //  fx - mu*fz <= 0
                                                     Eigen::Infinity,   //  fy + mu*fz <= infinity
                                                     0,                 //  fy - mu*fz <= 0
                                                     fz_max; //         //          fz <= fz_max    fz_max*contact = 0 or 1 depending on the contact
        }

        // Creating horizon bounds
        Eigen::VectorXd lower_bounds_horizon = Eigen::VectorXd::Zero(NUM_BOUNDS * LEGS * HORIZON_LENGTH);
        Eigen::VectorXd upper_bounds_horizon = Eigen::VectorXd::Zero(NUM_BOUNDS * LEGS * HORIZON_LENGTH);
        
        for (int i=0; i<HORIZON_LENGTH; i++)
        {
            lower_bounds_horizon.segment<NUM_BOUNDS * LEGS>(i*NUM_BOUNDS*LEGS) = lower_bounds;
            upper_bounds_horizon.segment<NUM_BOUNDS * LEGS>(i*NUM_BOUNDS*LEGS) = upper_bounds;
        }
        // std::cout<<"Lower bounds: \n"<<lower_bounds_horizon<<std::endl;
        // std::cout<<"Upper bounds: \n"<<upper_bounds_horizon<<std::endl;
    }

    /**
     * @brief This function calculates the gradient of the cost function
     * The mathematical proof for the derivation of this expression can be seen in the repport (and in miro)
     * 
     * @param[in] = Bqp_matrix , Q_matrix, R_matrix, U_vector, states, states_reference
     * @param[out] = gradient of the cost function
     * 
     * @returns = none
    */
    void setGradient()
    {
        gradient = (Bqp_matrix.transpose() * Q_matrix * Bqp_matrix + R_matrix)*U_vector + 2*Bqp_matrix.transpose() * Q_matrix * (Aqp_matrix * (states - states_reference));
  
        //std::cout << "Gradient: \n" << gradient << std::endl;
        //std::cout << "Shape: " << gradient.rows() << " x " << gradient.cols() << std::endl; 
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
    void setHessian()
    {   
        hessian = Bqp_matrix.transpose() * Q_matrix * Bqp_matrix + R_matrix;
    
        // std::cout << "Hessian: \n" << hessian << std::endl;
        // std::cout << "Shape: " << hessian.rows() << " x " << hessian.cols() << std::endl;

    }

    /** This function sets the initial guess for the solver.
    If the solver is running for the first time, the initial guess is a vector of zeros.
    If it is the second or higher iterations, the initial guess is a hot-start with the previous iteration's minimizer
    @param[in] = None
    @param[out] = The initial guess for the solver
    @returns = None
    */    
    // void setInitialGuess(){
    //     Eigen::VectorXd initial_guess = Eigen::VectorXd::Zero(NUM_STATE-1);
    //     if (is_first_run == false){
    //         //Retrieve the last minimizer correctly, for now it is a random vector
    //         //Goal is to have: initial_guess = last_minimizer
    //         initial_guess.setRandom();
    //     }

    //     std::cout << "Initial Guess: \n" << initial_guess << std::endl;
    // }

    void solveQP()
    {}

    void printResults()
    {}

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
    Eigen::Matrix<double,NUM_BOUNDS * LEGS, NUM_DOF> Ac_matrix;

    Eigen::Matrix<double, NUM_DOF * HORIZON_LENGTH, 1> gradient;
    Eigen::Matrix<double, NUM_DOF * HORIZON_LENGTH, NUM_DOF * HORIZON_LENGTH> hessian;

    

    bool is_first_run = true;  //to be set to false after first iteration, so that the initial guess is correctly set to hot-start the solver
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
    mpc.setAcMatrix();
    mpc.setBounds();
    mpc.setGradient();
    mpc.setHessian();
    // mpc.setInitialGuess();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;

    std::cout << "Time taken: " << duration.count() << " milliseconds" << std::endl;

    return 0;
}