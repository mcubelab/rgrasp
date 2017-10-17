cmake_minimum_required(VERSION 3.1.0)
set(CMAKE_CXX_STANDARD 11)
project(realsense_camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
  std_msgs
  image_transport
  cv_bridge
  pcl_conversions
  pcl_ros
)

find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

# OpenCV
find_package(OpenCV REQUIRED)
set(CMAKE_BUILD_TYPE "Release")
set (CMAKE_CXX_FLAGS "-Wno-unused-variable -g ${CMAKE_CXX_FLAGS}")

## Generate services in the 'srv' folder
add_service_files(
  FILES
  snapshot.srv
  start_lasers.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES vision_helper
        #CATKIN_DEPENDS   roscpp std_msgs sensor_msgs geometry_msgs message_generation
        #DEPENDS system_lib
)

add_library(vision_helper src/vision_helper.cpp)
add_dependencies(vision_helper ${PROJECT_NAME}_gencpp)
add_dependencies(vision_helper ${PROJECT_NAME}_generate_messages_cpp)


set(SOURCES
  src/stream.cpp
)
set(SOURCES_PROFILE
  src/profile_stream.cpp
)

set(HEADERS
#        src/stream.h
)

## Declare a C++ executable
include_directories(include ${catkin_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} /usr/local/include)
add_executable(stream src/stream.cpp ${HEADERS})
add_executable(profile_stream src/profile_stream.cpp ${HEADERS})
target_link_libraries(stream ${catkin_LIBRARIES} realsense ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(stream ${PROJECT_NAME}_gencpp)
add_dependencies(stream ${PROJECT_NAME}_generate_messages_cpp)


target_link_libraries(profile_stream ${catkin_LIBRARIES} realsense ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(profile_stream ${PROJECT_NAME}_gencpp)
add_dependencies(profile_stream ${PROJECT_NAME}_generate_messages_cpp)
