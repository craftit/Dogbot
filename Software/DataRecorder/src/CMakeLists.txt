cmake_minimum_required(VERSION 3.5)

set(CMAKE_BUILD_TYPE Debug)

set(DOGBOT_VERSION 0.1)
set(CMAKE_INSTALL_LIBDIR "/usr/local/lib")
set(CMAKE_INSTALL_INCLUDEDIR "/usr/local/include")

project(dogbot VERSION ${DOGBOT_VERSION} LANGUAGES CXX)# DESCRIPTION "API for React AI DogBot firmware interaction")
FIND_PACKAGE ( Threads REQUIRED )
FIND_PACKAGE ( DogBotAPI REQUIRED PATHS ../../API/build )

include (FindPkgConfig)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(JSONCPP jsoncpp)
  pkg_check_modules(PQXX libpqxx)
endif()

include_directories(${PQXX_INCLUDE_DIRS})
link_directories(${PQXX_LIBRARY_DIRS})

add_definitions(-std=c++11)

ADD_LIBRARY (DogBotDataRecorder STATIC
        DataRecorder.cc
)

set_target_properties(DogBotDataRecorder PROPERTIES VERSION ${PROJECT_VERSION})

target_link_libraries (DogBotDataRecorder LINK_PUBLIC DogBotAPI ${PQXX_LIBRARIES})

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(DogBotDataRecorder PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:include>
    /usr/local/include
    PRIVATE src)

add_executable (dogBotDataRecorder dogbotDataRecorder.cc)

target_link_libraries (dogBotDataRecorder LINK_PUBLIC DogBotDataRecorder ${CMAKE_THREAD_LIBS_INIT} )


# 'make install' to the correct locations (provided by GNUInstallDirs).

install(TARGETS DogBotDataRecorder EXPORT DogBotDataRecorderConfig
   ARCHIVE  DESTINATION lib
   LIBRARY  DESTINATION lib
   RUNTIME  DESTINATION bin)  # This is for Windows

install(DIRECTORY ../include/dogbot DESTINATION include)

INSTALL(TARGETS DogBotDataRecorder
        DESTINATION lib
        )
# INSTALL(FILES ../include/dogbot/DogBotAPI.hh
#         DESTINATION include/dogbot
#         )

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT DogBotDataRecorderConfig DESTINATION share/dogbot/cmake)

# This makes the project importable from the build directory
export(TARGETS DogBotDataRecorder FILE DogBotDataRecorderConfig.cmake)
