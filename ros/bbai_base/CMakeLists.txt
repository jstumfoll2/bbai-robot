cmake_minimum_required(VERSION 3.0.2)
project(bbai_base)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/base_node.py scripts/gps_node.py scripts/motor_map_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
