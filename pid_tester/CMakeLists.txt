cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

SET(PID_TEST_HEADERS src/PidTesterGui.h)
QT4_WRAP_CPP(PID_TEST_HEADERS_MOC ${PID_TEST_HEADERS})

SET(PID_TEST_FORMS src/pid_tester_gui.ui)
QT4_WRAP_UI(PID_TEST_FORMS_HEADERS ${PID_TEST_FORMS})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(pid_tester src/pid_tester.cpp
                                   src/PidTesterGui.cpp
                                   ${PID_TEST_HEADERS_MOC}
                                   ${PID_TEST_FORMS_HEADERS})
target_link_libraries(pid_tester ${QT_LIBRARIES})
