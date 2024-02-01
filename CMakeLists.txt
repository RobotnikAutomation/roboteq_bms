cmake_minimum_required(VERSION 2.8.3)
project(roboteq_bms)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  rcomponent
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    rospy
    std_msgs
    std_srvs
    rcomponent
  DEPENDS
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  src/${PROJECT_NAME}/roboteq_bms_ros.py
  src/${PROJECT_NAME}/roboteq_bms_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Modify this if launch or config folders are deleted
install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
