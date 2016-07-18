cmake_minimum_required(VERSION 2.8.3)
project(imu_3dm_gx3)

find_package(GeographicLib REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  geometry_utils
  asio_serial_device
)

add_message_files(
  FILES
  GPSFix.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp sensor_msgs geometry_utils asio_serial_device
  DEPENDS GeographicLib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GeographicLib_LIBRARY_DIRS}
)

add_definitions(
  ${catkin_DEFINITIONS}
  ${GeographicLib_DEFINITIONS}
)

add_executable(${PROJECT_NAME}
  src/imu_3dm_gx.cc
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
