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
  api_update.msg
  log.msg
)


add_service_files(
  FILES
        add_drone.srv
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
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories(
        ${catkin_LINK_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/natnet/lib
)

file(GLOB test_src
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.cpp"
)

# User API
add_library(MDP_API ${CMAKE_CURRENT_SOURCE_DIR}/src/user_api/user_api.cpp)
target_link_libraries(MDP_API ${catkin_LIBRARIES})
add_dependencies(MDP_API multi_drone_platform_generate_messages_cpp)

#Various libraries in MDP

add_library(LOGGER ${CMAKE_CURRENT_SOURCE_DIR}/src/debug/logger/logger.cpp)
target_link_libraries(LOGGER ${catkin_LIBRARIES})
add_dependencies(LOGGER multi_drone_platform_generate_messages_cpp)

add_library(COLLISION ${CMAKE_CURRENT_SOURCE_DIR}/src/drone_server/collision_management.cpp)
target_link_libraries(COLLISION ${catkin_LIBRARIES})
add_dependencies(COLLISION multi_drone_platform_generate_messages_cpp)

add_library(RIGIDBODY ${CMAKE_CURRENT_SOURCE_DIR}/src/drone_server/rigidbody.cpp)
target_link_libraries(RIGIDBODY COLLISION ${catkin_LIBRARIES})
add_dependencies(RIGIDBODY COLLISION multi_drone_platform_generate_messages_cpp)

add_library(TELEOP ${CMAKE_CURRENT_SOURCE_DIR}/src/teleop_control/teleop.cpp)
target_link_libraries(TELEOP ${catkin_LIBRARIES})
add_dependencies(TELEOP multi_drone_platform_generate_messages_cpp)

add_library(DEBUGAPP ${CMAKE_CURRENT_SOURCE_DIR}/src/debug/debug_app/debug_app.cpp)
target_link_libraries(DEBUGAPP ${catkin_LIBRARIES})
add_dependencies(DEBUGAPP multi_drone_platform_generate_messages_cpp)

add_library(DEBUGWINDOW ${CMAKE_CURRENT_SOURCE_DIR}/src/debug/debug_app/debug_window.cpp)
target_link_libraries(DEBUGWINDOW ${catkin_LIBRARIES})
add_dependencies(DEBUGWINDOW multi_drone_platform_generate_messages_cpp)



find_package(PkgConfig REQUIRED)
pkg_check_modules(GTKMM gtkmm-3.0)

link_directories(${GTKMM_LIBRARY_DIRS})
include_directories(include ${GTKMM_INCLUDE_DIRS})

# Programs and Bindings

add_executable(drone_server src/drone_server/drone_server.cpp)
target_link_libraries(drone_server ${catkin_LIBRARIES} COLLISION RIGIDBODY LOGGER)
add_dependencies(drone_server multi_drone_platform_generate_messages_cpp ${CMAKE_CURRENT_BINARY_DIR}/__wrappers.h)

add_executable(add_drone src/drone_server/add_drone.cpp)
target_link_libraries(add_drone ${catkin_LIBRARIES} COLLISION RIGIDBODY LOGGER)
add_dependencies(add_drone multi_drone_platform_generate_messages_cpp ${CMAKE_CURRENT_BINARY_DIR}/__wrappers.h)

add_executable(shutdown_drone_server src/drone_server/shutdown_drone_server.cpp)
target_link_libraries(shutdown_drone_server ${catkin_LIBRARIES})

add_executable(script_wk_10 scripts/demoWk10.cpp)
target_link_libraries(script_wk_10 ${catkin_LIBRARIES} MDP_API)

add_executable(follow_control scripts/follow_the_leader_demo.cpp)
target_link_libraries(follow_control ${catkin_LIBRARIES} MDP_API)

add_executable(test_program scripts/test_program_19Oct2019.cpp)
target_link_libraries(test_program ${catkin_LIBRARIES} MDP_API)

add_executable(velocity_example scripts/testVelControl.cpp)
target_link_libraries(velocity_example ${catkin_LIBRARIES} MDP_API)

add_executable(emergency_program src/user_api/emergency_program.cpp)
target_link_libraries(emergency_program ${catkin_LIBRARIES} MDP_API) 

add_executable(all_debug_windows src/debug/debug_app/debug_app_test.cpp)
target_link_libraries(all_debug_windows ${catkin_LIBRARIES} MDP_API ${GTKMM_LIBRARIES} DEBUGAPP DEBUGWINDOW)

add_executable(teleop_test src/teleop_control/teleop_test.cpp)
target_link_libraries(teleop_test ${catkin_LIBRARIES} MDP_API ${GTKMM_LIBRARIES} MDP_API TELEOP LOGGER)

add_executable(pos_example scripts/testPosControl.cpp)
target_link_libraries(pos_example ${catkin_LIBRARIES} MDP_API)
