cmake_minimum_required(VERSION 3.16)
project(orbslam3_x3_ros2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Threads REQUIRED)
get_filename_component(X3_PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(ORB_ROOT "${X3_PROJECT_ROOT}/src/ORB_SLAM3" CACHE PATH "Existing ORB-SLAM3 source")
if(NOT EXISTS "${ORB_ROOT}/lib/libORB_SLAM3.so")
  message(FATAL_ERROR "Build the existing ORB-SLAM3 core first: ${ORB_ROOT}")
endif()
add_executable(rgbd src/rgbd_node.cpp)
target_compile_options(rgbd PRIVATE -Wall -Wextra)
target_include_directories(rgbd PRIVATE ${ORB_ROOT} ${ORB_ROOT}/include
  ${ORB_ROOT}/include/CameraModels ${ORB_ROOT}/Thirdparty/Sophus
  ${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
ament_target_dependencies(rgbd rclcpp sensor_msgs geometry_msgs std_msgs message_filters)
target_link_libraries(rgbd ${ORB_ROOT}/lib/libORB_SLAM3.so
  ${ORB_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_ROOT}/Thirdparty/g2o/lib/libg2o.so
  ${OpenCV_LIBS} ${Pangolin_LIBRARIES} Eigen3::Eigen Threads::Threads)
set_target_properties(rgbd PROPERTIES INSTALL_RPATH
  "${ORB_ROOT}/lib;${ORB_ROOT}/Thirdparty/DBoW2/lib;${ORB_ROOT}/Thirdparty/g2o/lib;${X3_PROJECT_ROOT}/local/lib")
install(TARGETS rgbd DESTINATION lib/${PROJECT_NAME})
ament_package()
