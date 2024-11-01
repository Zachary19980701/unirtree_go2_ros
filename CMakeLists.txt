cmake_minimum_required(VERSION 3.16)
project(go2_ros)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")


find_package(unitree_sdk2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  sensor_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_filter
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${UNITREE_SDK2_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/go2_ros.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} unitree_sdk2)

add_executable(go2_images_node src/go2_images.cpp)
set_target_properties(go2_images_node PROPERTIES OUTPUT_NAME go2_images_node PREFIX "")
add_dependencies(go2_images_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(go2_images_node ${catkin_LIBRARIES} unitree_sdk2 ${OpenCV_LIBRARIES})
