cmake_minimum_required(VERSION 2.8.3)
project(cs_test)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
  gazebo_msgs
  std_msgs
  geometry_msgs
  message_generation
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  cpstate.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    # gazebo_ros_bumper
    cs_change
  DEPENDS 
    roscpp 
    gazebo_ros
    gazebo_msgs 
    std_msgs
    geometry_msgs
    message_runtime
)

add_library(cs_test src/cs_test.cpp)
# add_dependencies(gazebo_ros_bumper ${catkin_EXPORTED_TARGETS})
target_link_libraries(cs_test ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(cs_sub src/cs_subscribe.cpp)
# add_dependencies(cs_change ${catkin_EXPORTED_TARGETS})
target_link_libraries(cs_sub ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(cs_plot src/cs_plot.cpp)
target_link_libraries(cs_plot ${catkin_LIBRARIES})
add_executable(cs_pub src/cs_publish.cpp)
target_link_libraries(cs_pub ${catkin_LIBRARIES})
add_executable(cs_capacitance src/cs_capacitance.cpp)
target_link_libraries(cs_capacitance ${catkin_LIBRARIES})
add_executable(state_listener_test src/state_listener_test.cpp)
target_link_libraries(state_listener_test ${catkin_LIBRARIES})