cmake_minimum_required(VERSION 3.0.2)
project(imu_data)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs roscpp rospy std_msgs genmsg)

add_message_files(
  DIRECTORY
  msg
  FILES
  imu_data.msg
  imu_mag.msg
  imu_rpy.msg
  imu_tf.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(imu_mag src/imu_mag.cpp)
target_link_libraries(imu_mag ${catkin_LIBRARIES})

add_executable(imu_data src/imu_data.cpp)
target_link_libraries(imu_data ${catkin_LIBRARIES})

add_executable(imu_rpy src/imu_rpy.cpp)
target_link_libraries(imu_rpy ${catkin_LIBRARIES})

add_executable(imu_tf src/imu_tf.cpp)
target_link_libraries(imu_tf ${catkin_LIBRARIES})