cmake_minimum_required(VERSION 2.8.3)
project(realtime_mapping)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf2
)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})
include (FindPkgConfig)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(G2O)

include_directories(${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})
add_definitions(${G2O_DEFINITIONS})

include_directories(
  src/logic/
#  logic/include
#  SYSTEM ${catkin_INCLUDE_DIRS}
  SYSTEM $ENV{HAF_LIBS}/include/boost1_54
#  SYSTEM $ENV{HAF_COMMON}/include
#  SYSTEM $ENV{HAF_COMMON_BLACKBOX}/include
)

add_library(proj STATIC IMPORTED)
set_property(TARGET PROPERTY IMPORTED_LOCATION)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
#  LIBRARIES realtime_mapping
#  CATKIN_DEPENDS roscpp tf tf2
#  DEPENDS system_lib
)

add_executable(tfManager src/ros/tfManager.cpp)
target_link_libraries(tfManager ${catkin_LIBRARIES})
add_executable(pcManager src/ros/pcManager.cpp)
target_link_libraries(pcManager ${catkin_LIBRARIES} ${PCL_LIBRARIES})
