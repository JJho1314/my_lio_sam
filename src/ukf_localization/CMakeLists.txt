cmake_minimum_required(VERSION 3.0.2)
project(ukf_localization)

## Compile as C++11, supported in ROS Kinetic and newer


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  pcl_conversions
  geometry_msgs
  nav_msgs
  cv_bridge
)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    message(STATUS "find OpenMP")
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
else()
    message(STATUS "Coulid not find OpenMP")
endif()
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

catkin_package(
 INCLUDE_DIRS include
 DEPENDS PCL GTSAM

CATKIN_DEPENDS 
std_msgs
nav_msgs
geometry_msgs
sensor_msgs
message_runtime 
message_generation
#  LIBRARIES ukf_localization
#  CATKIN_DEPENDS roscpp rospy sensor_msg std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/../LIO-SAM/include
  include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include
)


link_directories(
  ${PROJECT_SOURCE_DIR}/../LIO-SAM/include
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)


add_executable(ukf_localization
  src/ukf_localization.cpp
)
add_dependencies(ukf_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ukf_localization
  ${catkin_LIBRARIES}
)

add_executable(ground_truth
  src/ground_truth.cpp
)
add_dependencies(ground_truth ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ground_truth
  ${catkin_LIBRARIES}
)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

install(
  TARGETS 
  ukf_localization ground_truth
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h" 
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch" 
)
