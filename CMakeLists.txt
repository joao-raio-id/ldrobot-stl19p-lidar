cmake_minimum_required(VERSION 3.5)
project(ldlidar C CXX)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

if(CMAKE_BUILD_TYPE MATCHES Release )
    #message(" * Release Mode")
    add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo )
    #message(" * Release with Debug Info Mode")
    add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug )
    message(" * Debug Mode")
endif()

list(APPEND CMAKE_MODULE_PATH
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules"
)

#############################################
# Dependencies

set(DEPENDENCIES
  rclcpp_lifecycle
  rclcpp
  rclcpp_components
  lifecycle_msgs
  std_msgs
  sensor_msgs
  diagnostic_msgs
  diagnostic_updater
)

####################find package#####################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/include")
find_package(udev REQUIRED)

############## LDLIDAR SDK START#####################################
# Build bundled LDLiDAR SDK as a subproject
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_sdk)
############## LDLIDAR SDK END#####################################

add_library(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_tools.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_component.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cmd_interface_linux.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lipkg.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tofbf.cpp
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${UDEV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ldlidar_driver
  ${UDEV_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  ${DEPENDENCIES}
)

#---------------------------------------------------------------------------------------
# generate executable and add libraries
#---------------------------------------------------------------------------------------
# Simple node
add_executable(${PROJECT_NAME}_node  
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_simple_node.cpp 
)

target_include_directories(${PROJECT_NAME}_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs
)

target_link_libraries(${PROJECT_NAME}_node 
  pthread
  ldlidar_driver
  ${UDEV_LIBRARIES}
)

# Lifecycle node
add_executable(${PROJECT_NAME}_lifecycle_node  
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_lifecycle_node.cpp 
)

target_include_directories(${PROJECT_NAME}_lifecycle_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(${PROJECT_NAME}_lifecycle_node 
  rclcpp 
  rclcpp_components 
  rclcpp_lifecycle
)

target_link_libraries(${PROJECT_NAME}_lifecycle_node 
  ${PROJECT_NAME}
)

#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------
install(TARGETS 
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node 
  ${PROJECT_NAME}_lifecycle_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  urdf
  launch 
  rviz2 
  scripts
  include
  config
  src
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})
ament_package()

