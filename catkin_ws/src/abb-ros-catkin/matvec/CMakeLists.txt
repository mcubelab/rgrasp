cmake_minimum_required(VERSION 2.4.6)
project(matvec)

ADD_CUSTOM_COMMAND(
  OUTPUT matvec.h
  COMMAND cat ${PROJECT_SOURCE_DIR}/src/Vec.h
              ${PROJECT_SOURCE_DIR}/src/Mat.h
              ${PROJECT_SOURCE_DIR}/src/RotMat.h 
              ${PROJECT_SOURCE_DIR}/src/Quaternion.h 
              ${PROJECT_SOURCE_DIR}/src/HomogTransf.h 
              ${PROJECT_SOURCE_DIR}/src/Polynom.h 
              > ${PROJECT_SOURCE_DIR}/include/matvec/matvec.h
  DEPENDS ${PROJECT_SOURCE_DIR}/src/Vec.h 
          ${PROJECT_SOURCE_DIR}/src/Mat.h 
          ${PROJECT_SOURCE_DIR}/src/RotMat.h 
          ${PROJECT_SOURCE_DIR}/src/Quaternion.h 
          ${PROJECT_SOURCE_DIR}/src/HomogTransf.h 
          ${PROJECT_SOURCE_DIR}/src/Polynom.h
  )

ADD_CUSTOM_TARGET(makeH ALL DEPENDS matvec.h)


add_library(matvec src/HomogTransf.cpp
                            src/Mat.cpp
                            src/Polynom.cpp
                            src/Quaternion.cpp
                            src/RotMat.cpp
                            src/Vec.cpp)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES matvec
   #CATKIN_DEPENDS   roscpp std_msgs sensor_msgs geometry_msgs message_generation
   #DEPENDS system_lib
)

## Mark executables and/or libraries for installation
install(TARGETS matvec
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

