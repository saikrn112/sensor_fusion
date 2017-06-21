cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion)
set (CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(BLAS REQUIRED)
find_package(Armadillo REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sensor_fusion
  CATKIN_DEPENDS nav_msgs roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)
## Declare a C++ library
# add_library(sensor_fusion
#   src/${PROJECT_NAME}/sensor_fusion.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(sensor_fusion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(sensor_fusion_node src/fusion_node.cpp src/fusion.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(sensor_fusion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(sensor_fusion_node
   ${catkin_LIBRARIES}
   ${ARMADILLO_LIBRARIES}
 )
