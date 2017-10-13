cmake_minimum_required(VERSION 3.1.0)
set(CMAKE_CXX_STANDARD 11)
project(passive_vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
  std_msgs
  image_transport
  realsense_camera
)
# find_package(catkin REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-Wno-unused-variable -g ${CMAKE_CXX_FLAGS}")

# OpenCV
find_package(OpenCV REQUIRED)

# CUDA
find_package(CUDA REQUIRED)
set(CUDA_HOST_COMPILER "c++")
set(CUDA_NVCC_FLAGS "-std=c++11 -g")
# set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_20,code=sm_20)

# find_package (Boost 1.54 REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  state.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  
)

set(SOURCES
  src/estimate.cpp
)

# set(HEADERS
#   include/internal.h
#   include/tsdf_volume.h
# )

# cuda_compile(HELPER_O src/kinfu.cu)

## Declare a C++ executable
include_directories(include ${catkin_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} /usr/local/include ${CUDA_INCLUDE_DIRS} /usr/include/eigen3)
# include_directories(include ${catkin_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} /usr/local/include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# add_executable(estimate ${SOURCES} ${HEADERS})
add_executable(estimate ${SOURCES} ${HELPER_O} ${HEADERS})
target_link_libraries(estimate ${catkin_LIBRARIES} realsense ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${OpenCV_LIBS} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_LIBRARIES})
# target_link_libraries(estimate ${catkin_LIBRARIES} realsense ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
add_dependencies(estimate ${PROJECT_NAME}_gencpp)
add_dependencies(estimate ${catkin_EXPORTED_TARGETS})
add_dependencies(estimate ${PROJECT_NAME}_generate_messages_cpp)

# cuda_add_executable(test_cuda src/test_cuda.cu)
# target_link_libraries(test_cuda ${catkin_LIBRARIES} realsense ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_LIBRARIES})
# add_dependencies(test_cuda ${PROJECT_NAME}_gencpp)
# add_dependencies(test_cuda ${PROJECT_NAME}_generate_messages_cpp)
