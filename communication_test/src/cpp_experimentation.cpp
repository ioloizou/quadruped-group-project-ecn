// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"

// eigen
#include <Eigen/Dense>

#include <iostream>
#include <chrono>

int main(){

    Eigen::Matrix<double, 12, 12> matrix = Eigen::Matrix<double, 12, 12>::Random(12, 12);
    matrix.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    matrix.block<3, 3>(5, 4) = Eigen::Matrix3d::Identity();

    auto start = std::chrono::high_resolution_clock::now();

    std::cout << "Random Matrix:" << std::endl;
    std::cout << matrix << std::endl;

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;

    std::cout << "Time taken: " << duration.count() << " milliseconds" << std::endl;

    return 0;
}