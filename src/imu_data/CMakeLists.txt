cmake_minimum_required(VERSION 3.0.2)
project(imu_data)

set(CXX_STANDARD 20)

add_subdirectory(Fusion)

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  geometry_msgs 
  roscpp 
  rospy 
  std_msgs 
  genmsg
)

find_package(TBB REQUIRED)

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

include_directories(include ${catkin_INCLUDE_DIRS} Fusion/include)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

add_executable(imu_mag src/imu_mag.cpp)
target_link_libraries(imu_mag ${catkin_LIBRARIES})

add_executable(imu_data src/imu_data.cpp)
target_link_libraries(imu_data ${catkin_LIBRARIES})

add_executable(imu_rpy src/imu_rpy.cpp)
target_link_libraries(imu_rpy ${catkin_LIBRARIES})

add_executable(imu_tf src/imu_tf.cpp)
target_link_libraries(imu_tf ${catkin_LIBRARIES})

add_executable(fusion_test src/Fusion_test.cpp)
target_link_libraries(fusion_test ${catkin_LIBRARIES} TBB::tbb)
target_compile_features(fusion_test PRIVATE cxx_std_20)