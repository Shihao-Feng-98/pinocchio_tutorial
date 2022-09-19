cmake_minimum_required(VERSION 3.0) 
project(PINOCCHIO_TUTORIAL) 

set(CMAKE_BUILD_TYPE Debug) # Debug/Release
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall") # g++ compile options

# using pkg-config find pinocchio
set(ENV{PKG_CONFIG_PATH} /opt/openrobots/lib/pkgconfig)
find_package(PkgConfig REQUIRED) 
pkg_check_modules(PIN REQUIRED pinocchio) # PIN 
message(STATUS "=== PIN_LIBRARIES: ${PIN_LIBRARIES}") # lib info
message(STATUS "=== PIN_INCLUDE_DIRS: ${PIN_INCLUDE_DIRS}") # pinocchio include path

# -L 
link_directories(/opt/openrobots/lib) # .so file path

add_executable(main main.cpp) # build executable file

# -l
target_link_libraries(main PRIVATE ${PIN_LIBRARIES}) # link .so file

target_include_directories(main PRIVATE ${PIN_INCLUDE_DIRS})
target_compile_options(main PRIVATE ${PIN_CFLAGS_OTHER})
