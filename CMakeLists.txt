cmake_minimum_required(VERSION 2.8.3)
project(rpwc_bridge)
set(CMAKE_BUILD_TYPE Release)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  #rpwc
  std_msgs
  #franka_gripper
  cv_bridge
  roslib
  pcl_conversions
  pcl_ros
  message_generation
  image_transport
  rpwc_msgs
  #zivid_camera
)


set(Poco_DIR cmake)
find_package(Poco REQUIRED COMPONENTS Foundation)
find_package(OpenCV)
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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# Generate services in the 'srv' folder
add_service_files(
  FILES
  setup_hardware.srv
  set_controller.srv
  set_controller_web.srv
  set_jog_mode_web.srv
  task_list.srv
  CameraData.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rpwc_bridge
#  CATKIN_DEPENDS franka_ros roscpp rospy rpwc std_msgs
#  DEPENDS system_lib
LIBRARIES ${Poco_LIBRARIES}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include/rpwc_bridge
  ${Poco_INCLUDE_DIRS}
)



## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rpwc_bridge.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/rpwc_bridge_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
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

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rpwc_bridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#questo decommentato
# add_executable(franka_bridge_node src/main_franka_bridge.cpp src/franka_bridge.cpp )
# add_dependencies(franka_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(franka_bridge_node ${catkin_LIBRARIES})

add_executable(summit_xls_steel_cl_bridge_node src/main_summit_xls_steel_cl_bridge.cpp src/summit_xls_steel_cl_bridge.cpp )
add_dependencies(summit_xls_steel_cl_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(summit_xls_steel_cl_bridge_node ${catkin_LIBRARIES})

add_executable(lidar_2D_bridge_node src/lidar_2D_bridge.cpp )
add_dependencies(lidar_2D_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_2D_bridge_node ${catkin_LIBRARIES})

# add_executable(franka_gripper_bridge_node src/main_franka_gripper_bridge.cpp src/franka_gripper_bridge.cpp)
# add_dependencies(franka_gripper_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(franka_gripper_bridge_node ${catkin_LIBRARIES})

add_executable(softhand_bridge_node src/main_softhand_bridge.cpp)
add_dependencies(softhand_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(softhand_bridge_node ${catkin_LIBRARIES})

add_executable(softclaw_bridge_node src/main_softclaw_bridge.cpp)
add_dependencies(softclaw_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(softclaw_bridge_node ${catkin_LIBRARIES})

add_executable(softhand_2_bridge_node src/main_softhand_2_bridge.cpp)
add_dependencies(softhand_2_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(softhand_2_bridge_node ${catkin_LIBRARIES})

# add_executable(softhand_industry_bridge_node src/main_softhand_industry_bridge.cpp)
# add_dependencies(softhand_industry_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(softhand_industry_bridge_node ${catkin_LIBRARIES})

add_executable(launch_from_web_node src/main_launch_from_web.cpp)
add_dependencies(launch_from_web_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(launch_from_web_node ${catkin_LIBRARIES} ${Poco_LIBRARIES})

add_executable(switch_ctr_from_web_node src/main_switch_ctr_from_web.cpp src/switch_ctr_from_web.cpp)
add_dependencies(switch_ctr_from_web_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(switch_ctr_from_web_node ${catkin_LIBRARIES} ${Poco_LIBRARIES})

add_executable(cam_from_web_node src/main_cam_from_web.cpp)
add_dependencies(cam_from_web_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cam_from_web_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(cam_to_web_node src/main_cam_to_web.cpp)
add_dependencies(cam_to_web_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cam_to_web_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable(ur_bridge_node src/main_ur_bridge.cpp src/ur_bridge.cpp )
# add_dependencies(ur_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(ur_bridge_node ${catkin_LIBRARIES})

add_executable(joy_web_node src/main_joy_web.cpp)
add_dependencies(joy_web_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_web_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(abb_bridge_node src/main_abb_bridge.cpp src/abb_bridge.cpp )
#add_dependencies(abb_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(abb_bridge_node ${catkin_LIBRARIES})

add_executable(request_from_web_node src/main_request_from_web.cpp src/request_from_web.cpp)
add_dependencies(request_from_web_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(request_from_web_node ${catkin_LIBRARIES} yaml-cpp)

# add_executable(test_abb_bridge_node src/main_test_abb_bridge.cpp src/test_abb_bridge.cpp src/arm_base_bridge.cpp)
# add_dependencies(test_abb_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(test_abb_bridge_node ${catkin_LIBRARIES})

# add_executable(zivid_bridge_node src/main_zivid_bridge.cpp src/zivid_bridge.cpp )
# add_dependencies(zivid_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(zivid_bridge_node ${catkin_LIBRARIES})


catkin_install_python(PROGRAMS scripts/ring_bridge.py scripts/summit_xls_steel_bridge_python.py  scripts/onboard_summit_xls_steel_bridge_python.py scripts/lidar_2D_bridge_python.py scripts/onboard_lidar_2D_bridge_python.py scripts/velTeachAndPlayCmd.py
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)