cmake_minimum_required(VERSION 3.14)
project(lidar_calibration)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)

# ── lidar_calibration_node ────────────────────────────────────────────────────
add_executable(lidar_calibration_node
  src/lidar_calibration_node.cpp
  src/LidarProcessor.cpp
  src/CalibrationProblem.cpp
  src/LidarCalibrator.cpp
  src/montecarlo_utils.cpp
)

target_include_directories(lidar_calibration_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
  /usr/local/include
)

ament_target_dependencies(lidar_calibration_node
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  rosbag2_cpp
)

target_link_libraries(lidar_calibration_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  gtsam
)

install(TARGETS lidar_calibration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
