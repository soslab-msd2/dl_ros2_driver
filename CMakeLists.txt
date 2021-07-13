cmake_minimum_required(VERSION 3.5)
project(dl_ros2_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)

add_library(dl_ros2_driver
    include/dl_ros2_driver.cpp
)

add_executable(${PROJECT_NAME}_node ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
    ${OpenCV_LIBRARIES}
    dl_ros2_driver
)
ament_target_dependencies(${PROJECT_NAME}_node 
    rclcpp
    sensor_msgs
)

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
install(
    DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
