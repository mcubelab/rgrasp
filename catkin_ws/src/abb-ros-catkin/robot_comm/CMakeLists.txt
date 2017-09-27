cmake_minimum_required(VERSION 2.8.3)
project(robot_comm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  matvec
)

add_message_files(DIRECTORY msg FILES
  robot_CartesianLog.msg
  robot_ForceLog.msg
  robot_JointsLog.msg
)


add_service_files(DIRECTORY srv FILES
    robot_ActivateCSS.srv
    robot_ActivateEGM.srv
    robot_AddBuffer.srv
    robot_AddJointPosBuffer.srv
    robot_Approach.srv
    robot_ClearBuffer.srv
    robot_ClearJointPosBuffer.srv
    robot_DeactivateCSS.srv
    robot_ExecuteBuffer.srv
    robot_ExecuteJointPosBuffer.srv
    robot_GetCartesian.srv
    robot_GetFK.srv
    robot_GetIK.srv
    robot_GetJoints.srv
    robot_GetState.srv
    robot_IsMoving.srv
    robot_Ping.srv
    robot_SetAcc.srv
    robot_SetCartesian.srv
    robot_SetCartesianJ.srv
    robot_SetComm.srv
    robot_SetDefaults.srv
    robot_SetInertia.srv
    robot_SetJoints.srv
    robot_SetMotionSupervision.srv
    robot_SetSpeed.srv
    robot_SetTool.srv
    robot_SetTrackDist.srv
    robot_SetWorkObject.srv
    robot_SetZone.srv
    robot_Stop.srv
    robot_IOSignal.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES robot_comm
   CATKIN_DEPENDS   roscpp std_msgs sensor_msgs geometry_msgs message_generation
   #DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(robot_comm
  src/robot_comm.cpp
)

target_link_libraries(robot_comm ${catkin_LIBRARIES})
add_dependencies(robot_comm robot_comm_generate_messages_cpp)

## Mark executables and/or libraries for installation
install(TARGETS robot_comm
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

