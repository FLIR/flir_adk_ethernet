cmake_minimum_required(VERSION 2.8.3)
project(flir_adk_ethernet)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  nodelet
  roscpp
  roslint
  sensor_msgs
  std_msgs
  message_generation
)

# -----------------
#     OpenCV
# -----------------
find_package(OpenCV REQUIRED)

# -----------------
#     Spinnaker
# -----------------
# Older Spinnaker library should be here: /usr/lib/libSpinnaker.so
# Newer Spinnaker library should be here: /opt/spinnaker/lib/libSpinnaker.so
find_library(SPINNAKER_LIBRARY Spinnaker REQUIRED HINTS /opt/spinnaker/lib)

# Older Spinnaker path is: /usr/include/spinnaker
# Newer Spinnaker path is: /opt/spinnaker/include/
find_path(SPINNAKER_INCLUDE_DIR CameraPtr.h REQUIRED HINTS /usr/include/spinnaker /opt/spinnaker/include)

message(STATUS "Spinnaker Library Path: ${SPINNAKER_LIBRARY}")
message(STATUS "Spinnaker Include Path: ${SPINNAKER_INCLUDE_DIR}")

# -----------------
#      Build
# -----------------
add_message_files(
  FILES
  MultiTimeHeader.msg
)

add_service_files(
  FILES
  GetNode.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  camera_info_manager
  image_transport
  cv_bridge
  nodelet
  sensor_msgs
  message_runtime
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${SPINNAKER_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node
  src/flir_boson_ethernet_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

# FILE(GLOB SpinLibs /usr/lib/libSpin*.so)

add_library(BosonCameraEthernet
  src/nodelets/BaseCameraController.cpp
  src/nodelets/CameraController.cpp
  src/nodelets/SyncCameraController.cpp
  src/nodelets/TimeSynchronizer.cpp
  src/nodelets/TimeDifference.cpp
  src/flir_adk_ethernet/EthernetCamera.cpp
  src/flir_adk_ethernet/ImageEventHandler.cpp
  src/flir_adk_ethernet/ImageFormat.cpp
  src/flir_adk_ethernet/Util.cpp
  spinnaker_wrappers/CameraListWrapper.cpp
  spinnaker_wrappers/SystemWrapper.cpp
  spinnaker_wrappers/CameraWrapper.cpp
)

add_dependencies(BosonCameraEthernet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(BosonCameraEthernet
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${SPINNAKER_LIBRARY}
)

set(ROSLINT_CPP_OPTS "--filter=-runtime/threadsafe_fn,-build/namespaces,-build/include_what_you_use")
roslint_cpp()

install(TARGETS ${PROJECT_NAME}_node BosonCameraEthernet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test
    test/test_flir_adk_ethernet.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test
    ${catkin_LIBRARIES}
    BosonCameraEthernet
  )

endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
