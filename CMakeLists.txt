cmake_minimum_required(VERSION 2.8.3)
project(serwo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  message_generation
  cv_bridge
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   SerwoInfo.msg
   ErrorInfo.msg
#   Message1.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serwo
#  CATKIN_DEPENDS roscpp rospy std_msgs
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(serwo
#   src/${PROJECT_NAME}/serwo.cpp
# )

## Declare a cpp executable
# add_executable(serwo_node src/serwo_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(serwo_node serwo_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(serwo_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS serwo serwo_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_serwo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(listener src/image_processing/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener serwo_generate_messages_cpp)

add_executable(image_process_diod src/y/image_process_diod.cpp)
target_link_libraries(image_process_diod ${catkin_LIBRARIES})
add_dependencies(image_process_diod serwo_generate_messages_cpp)

add_executable(grid_visualization src/rviz/grid_visualization.cpp)
target_link_libraries(grid_visualization ${catkin_LIBRARIES})
add_dependencies(grid_visualization serwo_generate_messages_cpp)

add_executable(grid_visualization_y src/y/grid_visualization_y.cpp)
target_link_libraries(grid_visualization_y ${catkin_LIBRARIES})
add_dependencies(grid_visualization_y serwo_generate_messages_cpp)

#add_executable(irp6_control src/irp6_control/irp6_control.cpp)
#target_link_libraries(irp6_control ${catkin_LIBRARIES})
#add_dependencies(irp6_control serwo_generate_messages_cpp)

#add_executable(irp6_control src/irp6_control/irp6_control.cpp)
#target_link_libraries(irp6_control ${catkin_LIBRARIES})
#add_dependencies(irp6_control force_control_msgs geometry_msgs)

add_executable(irp6_control_nowe src/irp6_control/irp6_control_nowe.cpp)
target_link_libraries(irp6_control_nowe ${catkin_LIBRARIES})
add_dependencies(irp6_control_nowe force_control_msgs geometry_msgs)

add_executable(conveyor_visualization src/rviz/conveyor_visualization.cpp)
target_link_libraries(conveyor_visualization ${catkin_LIBRARIES})
add_dependencies(conveyor_visualization serwo_generate_messages_cpp)

add_executable(irp6_serwo_y src/y/irp6_serwo_y.cpp)
target_link_libraries(irp6_serwo_y ${catkin_LIBRARIES})
add_dependencies(irp6_serwo_y force_control_msgs geometry_msgs)








