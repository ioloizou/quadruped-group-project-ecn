cmake_minimum_required(VERSION 3.0.2)
project(communication_test)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    quad_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(communication_node src/communication_node.cpp)
target_link_libraries(communication_node ${catkin_LIBRARIES})

add_executable(velpub src/velpub.cpp)
target_link_libraries(velpub ${catkin_LIBRARIES})

add_executable(grf_retriver src/grf_retriver.cpp)
target_link_libraries(grf_retriver ${catkin_LIBRARIES})

add_executable(localplansub src/localplansub.cpp)
target_link_libraries(localplansub ${catkin_LIBRARIES})