#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>

#include <cmath>
#include <iostream>
    
class MPC{
public:
    
    /**
     * @brief Sets the rotation matrix based on the given Euler angles.
     * The robot’s orientation is expressed as a vector of Z-Y-X Euler angles Θ = [φ θ ψ]ᵀ where ψ is the yaw, θ is the pitch, and φ is the roll.
     * 
     * @param euler_angles The Euler angles representing the rotation of the body respect to the inertial frame?
     */
    void setRotationMatrix(Eigen::Vector3d euler_angles){
        double psi = euler_angles(2);

        Eigen::Matrix3d R;
        // Rotation matrix
        R << cos(psi), -sin(psi), 0,
             sin(psi), cos(psi), 0,
             0, 0, 1;
    }

    void setQMatrix()
    {}

    void setRMatrix()
    {}

    void setAMatrixContinious()
    {}

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
private:

    
};


int main(){


    return 0;
}