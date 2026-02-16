cmake_minimum_required(VERSION 3.8)
project(duatic_cloud_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "rolling" OR "$ENV{ROS_DISTRO}" STREQUAL "kilted")
  add_compile_definitions(ROS_DISTRO_ROLLING)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)

add_executable(cloud_merger src/cloud_merger.cpp)

target_link_libraries(cloud_merger PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}

  # FIX: Jazzy doesn't have pcl_conversions::pcl_conversions target.
  ${pcl_conversions_LIBRARIES}

  laser_geometry::laser_geometry
  tf2::tf2
  tf2_ros::tf2_ros
  Eigen3::Eigen
  message_filters::message_filters

  pcl_ros::pcl_ros_tf
  pcl_ros::pcl_ros_filters
)

# Since we didn't link a target for pcl_conversions, we MUST manually add its includes
target_include_directories(cloud_merger PUBLIC
  ${pcl_conversions_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

install(TARGETS cloud_merger DESTINATION lib/${PROJECT_NAME})

ament_package()
