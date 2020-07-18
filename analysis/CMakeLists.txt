cmake_minimum_required(VERSION 3.11)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(slam_analysis)

# Get C or C++ sources in the current directory (only).
# file(GLOB C_SOURCES *.c)
# file(GLOB CPP_SOURCES *.cpp)
# set(SOURCES ${C_SOURCES} ${CPP_SOURCES})

# header only matplotlib-cpp goes here
include_directories(${CMAKE_SOURCE_DIR})
find_package(PythonLibs 2.7)


##### wheel odometry measurement visualization #######
add_executable(plot_wheel_odometry plot_wheel_odometry.cpp)
target_include_directories(plot_wheel_odometry PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(plot_wheel_odometry ${PYTHON_LIBRARIES})

##### lidar measurement visualization #######
add_executable(plot_lidar_measurement plot_lidar_measurement.cpp)
target_include_directories(plot_lidar_measurement PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(plot_lidar_measurement ${PYTHON_LIBRARIES})