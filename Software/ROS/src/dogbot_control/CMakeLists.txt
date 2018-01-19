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
)
#  ros_control_boilerplate

find_package(Eigen3 REQUIRED)


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
  CATKIN_DEPENDS 
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    urdf
    joint_limits_interface
    transmission_interface
    control_toolbox
    std_msgs
    sensor_msgs
    rosparam_shortcuts
  LIBRARIES
    dogbot_hw_interface
)
#   ros_control_boilerplate

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(JSONCPP jsoncpp)
  pkg_check_modules(LIBZMQ libzmq)
  pkg_check_modules(LIBUSB libusb-1.0)
  pkg_check_modules(DOGBOTAPI DogBotAPI)
endif()

include_directories(${JSONCPP_INCLUDE_DIRS})
link_directories(${JSONCPP_LIBRARY_DIRS})

include_directories(${LIBZMQ_INCLUDE_DIRS})
link_directories(${LIBZMQ_LIBRARY_DIRS})

include_directories(${LIBUSB_INCLUDE_DIRS})
link_directories(${LIBUSB_LIBRARY_DIRS})

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(DOGBOTAPI_LIBRARY /usr/local/lib/libDogBotAPI.a jsoncpp zmq usb-1.0)
set(DOGBOTAPI_INCLUDE /usr/local/include)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${DOGBOTAPI_INCLUDE}
)

link_directories(/opt/ros/kinetic/lib)
        
# DogBot Hardware Interface
add_library(dogbot_hw_interface
  src/dogbot_hw_interface.cpp
  src/dogbot_hw_control_loop.cpp
  src/sim_hw_interface.cpp
)
target_link_libraries(dogbot_hw_interface
  rosparam_shortcuts
  controller_manager
  urdf
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(dogbot_hw_main src/dogbot_hw_main.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(dogbot_hw_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(dogbot_hw_main
  dogbot_hw_interface
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
