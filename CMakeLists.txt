cmake_minimum_required(VERSION 3.0.2)
project(hokuyo3d)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
find_package(Boost 1.53 REQUIRED system serialization chrono thread)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

add_compile_options(-std=c++11)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include)

add_executable(hokuyo3d src/hokuyo3d.cpp)
add_executable(hokuyo_imu_to_livox_node src/imu_to_livox.cpp)
add_executable(imu_RC_filter_node src/imu_RC_filter.cpp)
add_executable(sub_pc_pub_odom_node src/sub_pc_pub_odom.cpp)
target_link_libraries(hokuyo3d ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(hokuyo_imu_to_livox_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(imu_RC_filter_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(sub_pc_pub_odom_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()

install(TARGETS hokuyo3d
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
