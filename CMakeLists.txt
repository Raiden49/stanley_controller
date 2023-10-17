cmake_minimum_required(VERSION 3.4.0)
project(stanley_controller)

find_package(catkin REQUIRED COMPONENTS
    nav_msgs
    roscpp
    rospy
    std_msgs
    tf
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME} SHARED
    "src/stanley_core.cpp"
    "src/stanley_tool.cpp"
)

add_executable(${PROJECT_NAME}_node
    src/stanley_controller.cpp 
)
target_link_libraries(${PROJECT_NAME}_node 
    ${PROJECT_NAME}     
    ${catkin_LIBRARIES}
)
