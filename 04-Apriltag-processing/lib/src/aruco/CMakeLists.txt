cmake_minimum_required(VERSION 3.0)

include_directories(aruco-3.1.12/src)
add_subdirectory(aruco-3.1.12)

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS python REQUIRED)
find_package(PythonInterp)
find_package(PythonLibs REQUIRED)

python_add_module(aruco_caller aruco_caller.cpp)
target_include_directories(aruco_caller PUBLIC
        ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
target_link_libraries(aruco_caller
        ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} ${OPENCV_LIBRARIES} ${BOOST_NUMPY_SO}
        aruco opencv_calib3d opencv_highgui)
