cmake_minimum_required(VERSION 2.8.3)
project(ackermann_vehicle_description)

#set c++11 compiler-flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


find_package(catkin REQUIRED)
catkin_package()
