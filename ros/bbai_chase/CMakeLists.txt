cmake_minimum_required(VERSION 3.0.2)
project(bbai_chase)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS scripts/chase_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
