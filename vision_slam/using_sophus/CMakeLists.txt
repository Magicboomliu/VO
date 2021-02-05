cmake_minimum_required(VERSION 3.0)
project(useSophus)

# 为使用 sophus，需要使用find_package命令找到它
find_package(Sophus REQUIRED)
find_package( OpenCV 3.4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Eigen
include_directories("/usr/include/eigen3")
add_executable(useSophus using_sophus.cpp)
target_link_libraries(useSophus Sophus::Sophus ${OpenCV_LIBS})
