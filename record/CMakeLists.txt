cmake_minimum_required(VERSION 3.8)
project(record)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(oak_dataset_recorder
  src/dataset_recorder.cpp
  src/dataset_recorder_node.cpp
  src/recorder_utils.cpp
  src/video_writer.cpp
)
target_compile_features(oak_dataset_recorder PUBLIC cxx_std_17)
target_include_directories(oak_dataset_recorder
  PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(oak_dataset_recorder ${OpenCV_LIBS})
ament_target_dependencies(oak_dataset_recorder
  cv_bridge
  nav_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS oak_dataset_recorder
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
