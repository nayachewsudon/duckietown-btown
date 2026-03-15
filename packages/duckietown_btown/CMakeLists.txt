cmake_minimum_required(VERSION 3.0.2)
project(duckietown_btown)

find_package(catkin REQUIRED)

catkin_package()

# Install launch files
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
