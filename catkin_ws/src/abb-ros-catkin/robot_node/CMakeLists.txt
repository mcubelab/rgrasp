cmake_minimum_required(VERSION 2.8.3)
project(robot_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  robot_comm
  matvec
)

catkin_package()

add_subdirectory(ABBInterpreter)

include_directories(src ABBInterpreter/include ${catkin_INCLUDE_DIRS})

add_executable(robot_node src/robot_node.cpp 
                        src/PracticalSocket/PracticalSocket.cpp
                        src/tinyxml/tinystr.cpp
                        src/tinyxml/tinyxml.cpp
                        src/tinyxml/tinyxmlerror.cpp
                        src/tinyxml/tinyxmlparser.cpp)
                        
target_link_libraries(robot_node ${catkin_LIBRARIES} ABBInterpreter)


