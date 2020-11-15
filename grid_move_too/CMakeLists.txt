cmake_minimum_required(VERSION 2.8.3)
project(grid_move_too)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(OpenCV_DIR /usr/share/OpenCV/)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  OpenCV
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  message_generation
  pcl_conversions
  pcl_msgs
  pcl_ros
  pr2_picknplace_msgs  
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(OpenCV 2 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES grid_move
#  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs image_geometry message_generation pcl_conversions pcl_msgs pcl_ros roscpp roslib sensor_msgs std_msgs tf visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(grid_move_too
#   src/${PROJECT_NAME}/grid_move_too.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(grid_move_too ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(grid_move_too src/grid_move_too.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(grid_move_too ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(grid_move_too
  ${catkin_LIBRARIES}
)
