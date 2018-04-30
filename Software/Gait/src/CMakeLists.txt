
cmake_minimum_required(VERSION 3.5)

set(CMAKE_BUILD_TYPE Debug)

set(DOGBOT_VERSION 0.1)
set(CMAKE_INSTALL_LIBDIR "/usr/local/lib")
set(CMAKE_INSTALL_INCLUDEDIR "/usr/local/include")

project(dogbot VERSION ${DOGBOT_VERSION} LANGUAGES CXX)# DESCRIPTION "Gait generator for React AI's DogBot")
FIND_PACKAGE ( Threads REQUIRED )

find_package(Eigen3 REQUIRED)
find_package(DogBotAPI REQUIRED)

add_definitions(-std=c++11)

ADD_LIBRARY (DogBotGait STATIC
        SplineCatmullRom.cc
        SimpleQuadrupedController.cc
        SplineGaitController.cc
)

set_target_properties(DogBotGait PROPERTIES VERSION ${PROJECT_VERSION})

target_link_libraries (DogBotGait LINK_PUBLIC )

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(DogBotGait PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:include>
    /usr/local/include
    PRIVATE src)

add_executable (testDogBotGait testDogBotGait.cc)
target_link_libraries (testDogBotGait LINK_PUBLIC DogBotGait )

add_executable (dogBotWalk dogBotWalk.cc)
target_link_libraries (dogBotWalk LINK_PUBLIC DogBotGait ${CMAKE_THREAD_LIBS_INIT} DogBotAPI)


# 'make install' to the correct locations (provided by GNUInstallDirs).

install(TARGETS DogBotGait EXPORT DogBotGaitConfig
   ARCHIVE  DESTINATION lib
   LIBRARY  DESTINATION lib
   RUNTIME  DESTINATION bin)  # This is for Windows

install(DIRECTORY ../include/dogbot DESTINATION include)

INSTALL(TARGETS DogBotGait
        DESTINATION lib
        )
# INSTALL(FILES ../include/dogbot/DogBotGait.hh
#         DESTINATION include/dogbot
#         )

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT DogBotGaitConfig DESTINATION lib/cmake/DogBotGait)

# This makes the project importable from the build directory
export(TARGETS DogBotGait FILE DogBotGaitConfig.cmake)
