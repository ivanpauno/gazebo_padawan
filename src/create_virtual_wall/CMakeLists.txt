cmake_minimum_required(VERSION 2.8.3)
project(create_virtual_wall)

add_compile_options(-std=c++11)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  roscpp
  std_msgs
  # ca_gazebo
)

catkin_package(
  CATKIN_DEPENDS gazebo_ros gazebo_plugins roscpp std_msgs
  LIBRARIES virtual_wall
  # ca_gazebo
)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
link_directories(${GAZEBO_LIBRARIES})

add_library(virtual_wall src/virtual_wall.cpp)
target_link_libraries(virtual_wall ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_include_directories(virtual_wall PUBLIC ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

install(TARGETS
  virtual_wall
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
