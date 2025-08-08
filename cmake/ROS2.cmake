cmake_minimum_required(VERSION 3.8)
project(seeker)

# ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Ceres REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(vpi 2.2 REQUIRED)
find_package(Eigen3 REQUIRED)

add_definitions(-DROS_AVAILABLE=2)

# 添加包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  /usr/local/cuda/include
  /usr/src/jetson_multimedia_api/include/
  /usr/src/jetson_multimedia_api/include/libjpeg-8b
  /usr/include/libdrm
)
list(APPEND ament_libraries
        rclcpp
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        cv_bridge
        image_transport
)
# 根据架构设置库目录
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86")
  set(LIB_DIR "libs/x86")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(LIB_DIR "libs/arm64")
endif()

link_directories(${LIB_DIR}
  /usr/local/cuda-11.4/lib64/
  /usr/lib/aarch64-linux-gnu/tegra
)

file(GLOB NVIDIA_SOURCE "/usr/src/jetson_multimedia_api/samples/common/classes/*.cpp")
list(FILTER NVIDIA_SOURCE EXCLUDE REGEX "NvEglRenderer\\.cpp$")
list(FILTER NVIDIA_SOURCE EXCLUDE REGEX "NvVulkanRenderer\\.cpp$")


# 生成可执行文件
add_executable(seeker_node
  src/seeker_ros2.cpp
  ${NVIDIA_SOURCE}
)

# 链接依赖库
target_include_directories(seeker_node
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(seeker_node
  ${rclcpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${stereo_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${image_transport_LIBRARIES}
  quad_undistort
  vpi
  seeker
  usb-1.0
  pthread nvv4l2 nvbufsurface nvbufsurftransform nvjpeg nvosd drm
  cuda cudart vulkan
)
ament_target_dependencies(seeker_node ${ament_libraries})

# 安装规则
install(TARGETS seeker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  ${LIB_DIR}/libseeker.so
  ${LIB_DIR}/libquad_undistort.so
  DESTINATION lib/
)

install(DIRECTORY 
  launch/ros2
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ./config/ DESTINATION share/${PROJECT_NAME}/config/)

ament_package()
