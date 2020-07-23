# ----------------------------------------------------------------------------
#   Basic Configuration
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.0)
project(aruco1 VERSION "3.1.12" LANGUAGES CXX)
set(PROJECT_SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11

INCLUDE_DIRECTORIES(aruco-3.1.12/src)
add_subdirectory(aruco-3.1.12)

find_package(PythonLibs REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
add_library(aruco_lib SHARED aruco_caller.cpp)
target_link_libraries(aruco_lib aruco opencv_calib3d opencv_highgui ${THREADLIB} ${PYTHON_LIBRARIES})