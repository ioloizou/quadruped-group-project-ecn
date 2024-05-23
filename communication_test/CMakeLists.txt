cmake_minimum_required(VERSION 3.0.2)
project(communication_test)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    quad_msgs
)

find_package(Eigen3 REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
)
include_directories(/usr/local/include/osqp)
find_package(OsqpEigen REQUIRED)


catkin_package(
    DEPENDS EIGEN3
    CATKIN_DEPENDS roscpp std_msgs quad_msgs
    LIBRARIES communication_test
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    /usr/local/include/coin-or  # Replace this with the correct path
)



add_executable(communication_node src/communication_node.cpp)
target_link_libraries(communication_node ${catkin_LIBRARIES})

add_executable(velpub src/velpub.cpp)
target_link_libraries(velpub ${catkin_LIBRARIES})

add_executable(grf_retriver src/grf_retriver.cpp)
target_link_libraries(grf_retriver ${catkin_LIBRARIES})

add_executable(localplansub src/localplansub.cpp)
target_link_libraries(localplansub ${catkin_LIBRARIES})

# add_executable(main src/ipopt_example_ioannis/main.cpp)
# target_link_libraries(main ${catkin_LIBRARIES})


# link_directories(/usr/local/lib)
# target_link_libraries(main ipopt)

add_executable(osqp_training src/osqp_example_ioannis/osqp_training.cpp)
target_link_libraries(osqp_training ${catkin_LIBRARIES})
target_link_libraries(osqp_training OsqpEigen::OsqpEigen)

add_executable(my_osqp src/osqp_example_ioannis/my_osqp.cpp)
target_link_libraries(my_osqp ${catkin_LIBRARIES})
target_link_libraries(my_osqp OsqpEigen::OsqpEigen)

add_executable(cpp_experimentation src/cpp_experimentation.cpp)
target_link_libraries(cpp_experimentation ${catkin_LIBRARIES})
target_link_libraries(cpp_experimentation OsqpEigen::OsqpEigen)

