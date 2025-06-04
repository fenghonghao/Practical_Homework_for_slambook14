cmake_minimum_required(VERSION 3.11)

project(homework)

find_package(Eigen3 REQUIRED)

add_executable(task task.cpp)
target_link_libraries(task Eigen3::Eigen)