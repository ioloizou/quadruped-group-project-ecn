cmake_minimum_required(VERSION 3.0.2)
project(lmpc_controller)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    quad_msgs
    quad_utils
)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    /usr/local/include/osqp
    ${OsqpEigen_INCLUDE_DIRS}
    include
)

# Declare a C++ library
add_library(lmpc_controller
    src/lmpc_controller.cpp
    #add other source files here
)

target_link_libraries(lmpc_controller 
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
)

add_dependencies(lmpc_controller
    ${lmpc_controller_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} lmpc_controller
    CATKIN_DEPENDS roscpp std_msgs quad_msgs quad_utils
    DEPENDS EIGEN3 OsqpEigen
)

add_executable(linear_mpc src/lmpc_controller.cpp)
target_link_libraries(linear_mpc 
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
)

install(TARGETS linear_mpc lmpc_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)