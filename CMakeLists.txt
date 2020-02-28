cmake_minimum_required(VERSION 2.8.3)
project(multi_drone_platform)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


add_custom_command(
  OUTPUT 
      ${CMAKE_CURRENT_SOURCE_DIR}/src/drone_server/wrappers.h
      ${CMAKE_CURRENT_BINARY_DIR}/__wrappers.h
  COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/src/drone_server/gen_wrappers_h.py
)

add_custom_target(
  always_run_target ALL
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/__wrappers.h
)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  roscpp
  rospy
  rviz
  std_msgs
  crazyflie_driver
)

add_message_files(
  FILES
  apiUpdate.msg
  log.msg
)


add_service_files(
  FILES
)



generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs

)

###################################
## catkin specific configuration ##
###################################


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multi_drone_platform
  CATKIN_DEPENDS 
  geometry_msgs
  message_generation
  sensor_msgs
  roscpp
  rospy
  rviz
  std_msgs
  crazyflie_ros
#  DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB obj_src
  "${CMAKE_CURRENT_SOURCE_DIR}/src/objects/*.cpp"
)
file(GLOB test_src
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.cpp"
)
# Declare a C++ library
add_library(LOGGER
  ${CMAKE_CURRENT_SOURCE_DIR}/src/debug/logger/logger.cpp
)
target_link_libraries(LOGGER ${catkin_LIBRARIES})
add_dependencies(LOGGER multi_drone_platform_generate_messages_cpp)

add_library(RIGIDBODY
  ${CMAKE_CURRENT_SOURCE_DIR}/src/objects/rigidBody.cpp
)
target_link_libraries(RIGIDBODY ${catkin_LIBRARIES})
add_dependencies(RIGIDBODY multi_drone_platform_generate_messages_cpp)


add_library(DEBUGAPP
  ${CMAKE_CURRENT_SOURCE_DIR}/src/debug/debug_window/debugApplication.cpp
)
add_library(DEBUGWINDOW
  ${CMAKE_CURRENT_SOURCE_DIR}/src/debug/debug_window/debugUI.cpp
)
target_link_libraries(DEBUGWINDOW ${catkin_LIBRARIES})
add_dependencies(DEBUGWINDOW multi_drone_platform_generate_messages_cpp)

target_link_libraries(DEBUGAPP ${catkin_LIBRARIES})
add_dependencies(DEBUGAPP multi_drone_platform_generate_messages_cpp)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTKMM gtkmm-3.0)

add_library(MDP_API
  ${CMAKE_CURRENT_SOURCE_DIR}/src/user_api/user_api.cpp
)
target_link_libraries(MDP_API ${catkin_LIBRARIES})
add_dependencies(MDP_API multi_drone_platform_generate_messages_cpp)


link_directories(${GTKMM_LIBRARY_DIRS})
include_directories(include ${GTKMM_INCLUDE_DIRS})

# Programs and Bindings

add_executable(drone_server src/drone_server/drone_server.cpp)
target_link_libraries(drone_server ${catkin_LIBRARIES} RIGIDBODY LOGGER) 
add_dependencies(drone_server multi_drone_platform_generate_messages_cpp ${CMAKE_CURRENT_BINARY_DIR}/__wrappers.h)

add_executable(script scripts/demoWk10.cpp)
target_link_libraries(script ${catkin_LIBRARIES} MDP_API)

add_executable(follow scripts/follow_the_leader_demo.cpp)
target_link_libraries(follow ${catkin_LIBRARIES} MDP_API)

add_executable(ps4teleop src/PS4_control/ps4teleop.cpp)
target_link_libraries(ps4teleop ${catkin_LIBRARIES} MDP_API) 

add_executable(tst_program scripts/test_program_19Oct2019.cpp)
target_link_libraries(tst_program ${catkin_LIBRARIES} MDP_API) 

add_executable(emergency_program src/user_api/emergency_program.cpp)
target_link_libraries(emergency_program ${catkin_LIBRARIES} MDP_API) 

add_executable(debug src/debug/debug_window/debugNode.cpp)
target_link_libraries(debug ${catkin_LIBRARIES} MDP_API ${GTKMM_LIBRARIES} DEBUGAPP DEBUGWINDOW RIGIDBODY LOGGER) 

add_executable(debugApp src/debug/debug_window/debugAppTest.cpp)
target_link_libraries(debugApp ${catkin_LIBRARIES} MDP_API ${GTKMM_LIBRARIES} DEBUGAPP DEBUGWINDOW RIGIDBODY LOGGER) 
