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
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
)

link_directories(
  ${catkin_LINK_DIRS}
)

# User API
add_library(MDP_API
        src/user_api/user_api.cpp)
target_link_libraries(MDP_API ${catkin_LIBRARIES})
add_dependencies(MDP_API multi_drone_platform_generate_messages_cpp)

#Various libraries in MDP

add_library(LOGGER
        src/debug/logger/logger.cpp)
target_link_libraries(LOGGER ${catkin_LIBRARIES})
add_dependencies(LOGGER multi_drone_platform_generate_messages_cpp)

add_library(KD_TREE ${CMAKE_CURRENT_SOURCE_DIR}/src/icp_implementation/kd_tree_3d.cpp)
target_link_libraries(KD_TREE ${catkin_LIBRARIES})
add_dependencies(KD_TREE multi_drone_platform_generate_messages_cpp)

add_library(ICP_OBJ ${CMAKE_CURRENT_SOURCE_DIR}/src/icp_implementation/icp_object.cpp)
target_link_libraries(ICP_OBJ ${catkin_LIBRARIES})
add_dependencies(ICP_OBJ multi_drone_platform_generate_messages_cpp)

add_library(ICP_IMPL ${CMAKE_CURRENT_SOURCE_DIR}/src/icp_implementation/icp_impl.cpp)
target_link_libraries(ICP_IMPL ${catkin_LIBRARIES} KD_TREE)
add_dependencies(ICP_IMPL multi_drone_platform_generate_messages_cpp)

add_library(COLLISION
        src/collision_management/static_physical_management.cpp
        src/collision_management/potential_fields.cpp
        )
target_link_libraries(COLLISION ${catkin_LIBRARIES})
add_dependencies(COLLISION multi_drone_platform_generate_messages_cpp)

add_library(RIGIDBODY
        src/drone_server/rigidbody.cpp)
target_link_libraries(RIGIDBODY COLLISION ICP_OBJ ${catkin_LIBRARIES})
add_dependencies(RIGIDBODY COLLISION multi_drone_platform_generate_messages_cpp)

add_library(TELEOP
        src/teleop_control/teleop.cpp)
target_link_libraries(TELEOP ${catkin_LIBRARIES})
add_dependencies(TELEOP multi_drone_platform_generate_messages_cpp)

add_library(DEBUGAPP
        src/debug/debug_app/debug_app.cpp
        src/debug/debug_app/debug_window.cpp
        )
target_link_libraries(DEBUGAPP ${catkin_LIBRARIES})
add_dependencies(DEBUGAPP multi_drone_platform_generate_messages_cpp)




find_package(PkgConfig REQUIRED)
pkg_check_modules(GTKMM gtkmm-3.0)

link_directories(${GTKMM_LIBRARY_DIRS})
include_directories(include ${GTKMM_INCLUDE_DIRS})

# Programs and Bindings

add_executable(drone_server src/drone_server/drone_server.cpp)
target_link_libraries(drone_server ${catkin_LIBRARIES} COLLISION ICP_IMPL RIGIDBODY LOGGER)
add_dependencies(drone_server multi_drone_platform_generate_messages_cpp ${CMAKE_CURRENT_BINARY_DIR}/__wrappers.h)

add_executable(add_drone src/drone_server/add_drone.cpp)
target_link_libraries(add_drone ${catkin_LIBRARIES} COLLISION RIGIDBODY LOGGER)
add_dependencies(add_drone multi_drone_platform_generate_messages_cpp ${CMAKE_CURRENT_BINARY_DIR}/__wrappers.h)

add_executable(shutdown_drone_server src/drone_server/shutdown_drone_server.cpp)
target_link_libraries(shutdown_drone_server ${catkin_LIBRARIES})

add_executable(emergency_program src/drone_server/emergency_program.cpp)
target_link_libraries(emergency_program ${catkin_LIBRARIES} LOGGER)

add_executable(all_debug_windows src/debug/debug_app/debug_app_test.cpp)
target_link_libraries(all_debug_windows ${catkin_LIBRARIES} MDP_API ${GTKMM_LIBRARIES} DEBUGAPP)

add_executable(teleop_test src/teleop_control/teleop_test.cpp)
target_link_libraries(teleop_test ${catkin_LIBRARIES} MDP_API ${GTKMM_LIBRARIES} MDP_API TELEOP LOGGER)

file(GLOB files  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.cpp")
foreach(file ${files})
    get_filename_component(exe_name "${file}" NAME_WE)
    add_executable(${exe_name} ${file})
    target_link_libraries(${exe_name} ${catkin_LIBRARIES} MDP_API)
endforeach()

option(BUILD_DOC "Build documentation" ON)

# check if Doxygen is installed
find_package(Doxygen)
if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR})

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target(ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM )

else (DOXYGEN_FOUND)
    message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)

