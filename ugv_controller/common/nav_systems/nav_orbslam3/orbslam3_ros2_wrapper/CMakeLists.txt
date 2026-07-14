cmake_minimum_required(VERSION 3.8)
project(orbslam3_ros2_wrapper)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Where the ORB_SLAM3 source checkout lives (built once by ../build.sh). Override
# with -DORB_SLAM3_DIR=... if you keep it elsewhere.
set(ORB_SLAM3_DIR "/home/joshroq/projects/slam_source/ORB_SLAM3" CACHE PATH
    "Path to the ORB_SLAM3 source checkout (with lib/libORB_SLAM3.so built)")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

add_executable(rgbd_odom_node src/rgbd_odom_node.cpp)

target_include_directories(rgbd_odom_node PRIVATE
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)

target_link_libraries(rgbd_odom_node
  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  -lboost_system
)

ament_target_dependencies(rgbd_odom_node
  rclcpp cv_bridge sensor_msgs geometry_msgs message_filters tf2 tf2_ros tf2_eigen)

install(TARGETS rgbd_odom_node DESTINATION lib/${PROJECT_NAME})

ament_package()
