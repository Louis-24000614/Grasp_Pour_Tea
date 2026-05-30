cmake_minimum_required(VERSION 3.8)
project(camera_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(REALSENSE2 QUIET realsense2)

if(REALSENSE2_FOUND)
  set(REALSENSE2_INCLUDE_DIRS ${REALSENSE2_INCLUDE_DIRS})
  set(REALSENSE2_LIBRARY_DIRS ${REALSENSE2_LIBRARY_DIRS} ${REALSENSE2_LIBDIR})

  # Prefer absolute library path to avoid linker search-path issues on arm64 ROS installs.
  find_library(REALSENSE2_LIBRARY NAMES realsense2
    HINTS
      ${REALSENSE2_LIBRARY_DIRS}
      /opt/ros/humble/lib/aarch64-linux-gnu
      /opt/ros/$ENV{ROS_DISTRO}/lib/aarch64-linux-gnu
      /usr/lib/aarch64-linux-gnu
      /usr/local/lib
  )
  if(REALSENSE2_LIBRARY)
    set(REALSENSE2_LIBRARIES ${REALSENSE2_LIBRARY})
  else()
    set(REALSENSE2_LIBRARIES ${REALSENSE2_LIBRARIES})
  endif()
else()
  find_path(REALSENSE2_INCLUDE_DIRS librealsense2/rs.hpp
    HINTS
      /usr/include
      /usr/local/include
      /opt/ros/humble/include
      /opt/ros/$ENV{ROS_DISTRO}/include
  )
  find_library(REALSENSE2_LIBRARY NAMES realsense2
    HINTS
      /usr/lib
      /usr/lib/aarch64-linux-gnu
      /usr/local/lib
      /opt/ros/humble/lib
      /opt/ros/humble/lib/aarch64-linux-gnu
      /opt/ros/$ENV{ROS_DISTRO}/lib
      /opt/ros/$ENV{ROS_DISTRO}/lib/aarch64-linux-gnu
  )

  if(NOT REALSENSE2_INCLUDE_DIRS OR NOT REALSENSE2_LIBRARY)
    message(FATAL_ERROR
      "librealsense2 not found. Install Intel RealSense SDK development files "
      "(for Ubuntu usually: sudo apt install librealsense2-dev).")
  endif()

  set(REALSENSE2_LIBRARIES ${REALSENSE2_LIBRARY})
endif()

add_executable(dep_cam_node dep_cam.cpp)
target_include_directories(dep_cam_node PRIVATE ${REALSENSE2_INCLUDE_DIRS})
if(REALSENSE2_LIBRARY_DIRS)
  target_link_directories(dep_cam_node PRIVATE ${REALSENSE2_LIBRARY_DIRS})
endif()
target_link_libraries(dep_cam_node ${REALSENSE2_LIBRARIES})
ament_target_dependencies(dep_cam_node rclcpp sensor_msgs)

install(TARGETS dep_cam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
