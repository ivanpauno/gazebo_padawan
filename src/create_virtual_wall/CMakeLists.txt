cmake_minimum_required(VERSION 2.8.3)
project(create_virtual_wall)

add_compile_options(-std=c++11)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  # ca_gazebo
)

catkin_package(
  CATKIN_DEPENDS gazebo_ros gazebo_plugins
  # ca_gazebo
)

install(DIRECTORY launch worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
