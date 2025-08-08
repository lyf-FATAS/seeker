cmake_minimum_required(VERSION 3.0.2)
project(seeker)

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  image_transport
  sensor_msgs
  cv_bridge
)
find_package(vpi 2.2 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES seeker_nodelet
  CATKIN_DEPENDS 
    roscpp 
    nodelet 
    image_transport 
    sensor_msgs 
)

# 添加包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
  /usr/local/cuda/include
  /usr/src/jetson_multimedia_api/include/
  /usr/src/jetson_multimedia_api/include/libjpeg-8b
  /usr/include/libdrm
)

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

# 生成共享库
add_library(seeker_nodelet SHARED
  src/seeker_nodelet.cpp
  ${NVIDIA_SOURCE}
)

# 链接依赖库
target_link_libraries(seeker_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  quad_undistort
  vpi
  seeker
  usb-1.0
  pthread nvv4l2 nvbufsurface nvbufsurftransform nvjpeg nvosd drm
  cuda cudart vulkan
)

add_executable(seeker_node src/seeker_node.cpp)
target_link_libraries(seeker_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

# 安装规则
install(TARGETS seeker_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
