cmake_minimum_required(VERSION 3.10) 
project(PinocchinoTutorial) 

set(CMAKE_BUILD_TYPE Debug) # Debug/Release
set(CMAKE_CXX_STANDARD 11) # c++11

find_package(pinocchio REQUIRED)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE pinocchio::pinocchio) # -l