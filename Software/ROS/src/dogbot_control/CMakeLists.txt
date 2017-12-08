cmake_minimum_required(VERSION 2.8.3)
project(dogbot_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  ros_control_boilerplate
)

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
  INCLUDE_DIRS include
#  LIBRARIES ROSDogBot
  CATKIN_DEPENDS 
   ros_control_boilerplate
   roscpp
    
#  DEPENDS system_lib
)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(JSONCPP jsoncpp)
  pkg_check_modules(LIBZMQ libzmq)
endif()

include_directories(${JSONCPP_INCLUDE_DIRS})
link_directories(${JSONCPP_LIBRARY_DIRS})

include_directories(${LIBZMQ_INCLUDE_DIRS})
link_directories(${LIBZMQ_LIBRARY_DIRS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#FIND_LIBRARY(DOGBOTAPI_LIBRARY DogBotAPI /home/charles/src/active/BMC2-Firmware/API/build)

# There has got to be a better way of doing this, but later.
set(DOGBOTAPI_INSTALL /home/charles/src/active/BMC2-Firmware/API)
set(DOGBOTAPI_LIBRARY ${DOGBOTAPI_INSTALL}/build/src/libDogBotAPI.a jsoncpp zmq)
set(DOGBOTAPI_INCLUDE ${DOGBOTAPI_INSTALL}/include)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${DOGBOTAPI_INCLUDE}
)

link_directories(/opt/ros/kinetic/lib)
        
# DogBot Hardware Interface
add_library(dogbot_hw_interface
  src/dogbot_hw_interface.cpp
)
target_link_libraries(dogbot_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(dogbot_hw_main src/dogbot_hw_main.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(dogbot_hw_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(dogbot_hw_main
  dogbot_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
  ${DOGBOTAPI_LIBRARY}
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(dogbot_hw_main PROPERTIES OUTPUT_NAME dogbot_hw_main PREFIX "")


## Install ------------------------------------------------------------

# Install libraries
#install(TARGETS
#    dogbot_hw_interface
#  LIBRARY DESTINATION
#    ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

# Install executables
#install(TARGETS
#    dogbot_hw_main
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# Install header files
#install(DIRECTORY include/dogbot_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
