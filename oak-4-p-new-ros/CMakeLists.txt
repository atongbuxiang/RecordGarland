cmake_minimum_required (VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS
project(ros2_oak_ffc_sync VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-g)

## is used, also find other catkin packages
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc imgcodecs videoio highgui)
if(NOT OpenCV_FOUND)
  set(_opencv_version 3)
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc imgcodecs videoio highgui)
endif()

find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(dependencies
  camera_info_manager
  cv_bridge
  depthai_ros_msgs
  depthai_bridge
  nav_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  stereo_msgs
  std_msgs
  vision_msgs
)

macro(dai_add_node_ros2 node_name node_src)
  add_executable("${node_name}" "${node_src}")

target_link_libraries("${node_name}"
                        depthai::core
                        opencv_imgproc
                        opencv_imgcodecs
                        opencv_videoio
                        opencv_highgui
                        yaml-cpp)

  ament_target_dependencies("${node_name}"
                            ${dependencies})

endmacro()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

dai_add_node_ros2(oak_ffc_sync src/oak_ffc_sync_publisher.cpp)
target_sources(oak_ffc_sync PRIVATE src/oak_dataset_recorder.cpp)

add_library(oak_ffc_sync_component SHARED src/oak_ffc_sync_publisher.cpp)
target_sources(oak_ffc_sync_component PRIVATE src/oak_dataset_recorder.cpp)
target_compile_definitions(oak_ffc_sync_component PRIVATE OAK_FFC_SYNC_COMPONENT_ONLY)
target_link_libraries(oak_ffc_sync_component
                      depthai::core
                      opencv_imgproc
                      opencv_imgcodecs
                      opencv_videoio
                      opencv_highgui
                      yaml-cpp)
ament_target_dependencies(oak_ffc_sync_component ${dependencies})
rclcpp_components_register_nodes(oak_ffc_sync_component "OakFfcSyncNode")

## Install these nodes
install(TARGETS
        oak_ffc_sync
        oak_ffc_sync_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
FILES_MATCHING PATTERN "*.py")

ament_package()
