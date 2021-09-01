cmake_minimum_required(VERSION 2.8.3)
project(scha_imu_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scha_imu_node src/scha_imu_node.cpp)
target_link_libraries(scha_imu_node ${catkin_LIBRARIES} )

