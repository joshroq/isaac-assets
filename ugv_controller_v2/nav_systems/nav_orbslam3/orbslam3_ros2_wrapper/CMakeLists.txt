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

if(NOT EXISTS "${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so")
  message(FATAL_ERROR
    "libORB_SLAM3.so not found under ${ORB_SLAM3_DIR}/lib.\n"
    "Build the ORB-SLAM3 source first — ../build.sh does it for you.")
endif()

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

# ORB-SLAM3's libraries live OUTSIDE the ament install tree (they are linked by absolute
# path straight from the source checkout) and ament strips the build RPATH on install.
# Without an explicit install RPATH the installed node dies at startup with
#   "libORB_SLAM3.so: cannot open shared object file: No such file or directory"
# unless the caller happens to export LD_LIBRARY_PATH. Bake the paths in instead, so
# `ros2 run` / `ros2 launch` just work.
set_target_properties(rgbd_odom_node PROPERTIES
  INSTALL_RPATH "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib"
  INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS rgbd_odom_node DESTINATION lib/${PROJECT_NAME})

ament_package()
