cmake_minimum_required(VERSION 2.8.3)
project(rosopenigtbridge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenIGTLink REQUIRED)
include(${OpenIGTLink_USE_FILE})



find_package(catkin REQUIRED COMPONENTS
    std_msgs
    sensor_msgs
    image_transport
    roscpp
    geometry_msgs
    cmake_modules
    tf
    kuka_rsi_driver
 )
FIND_PACKAGE(Eigen REQUIRED)
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

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
  INCLUDE_DIRS include
#  LIBRARIES rosopenigtbridge
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    std_msgs
    kuka_rsi_driver
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(rosopenigtbridge
#   src/${PROJECT_NAME}/rosopenigtbridge.cpp
# )

## Declare a cpp executable
add_executable(ros2openigtlink src/ros2oigtlGenericConverter.cpp include/ros2oigtlGenericConverter.h src/ros2openigtlink.cpp include/ros2oigtlbridge.h src/ros2oigtlbridge.hxx include/ros2oigtlDefaultConversions.h src/ros2oigtlDefaultConversions.cpp)
add_executable(ros2oigtlPose src/ros2oigtlGenericConverter.cpp include/ros2oigtlGenericConverter.h src/ros2oigtlPose.cpp include/ros2oigtlDefaultConversions.h src/ros2oigtlDefaultConversions.cpp)
add_executable(ros2oigtlTransformation src/ros2oigtlGenericConverter.cpp include/ros2oigtlGenericConverter.h src/ros2oigtlTransformation.cpp include/ros2oigtlDefaultConversions.h src/ros2oigtlDefaultConversions.cpp)
add_executable(ros2oigtlImage src/ros2oigtlGenericConverter.cpp include/ros2oigtlGenericConverter.h src/ros2oigtlImage.cpp include/ros2oigtlDefaultConversions.h src/ros2oigtlDefaultConversions.cpp)
add_executable(ros2oigtlMesh src/ros2oigtlGenericConverter.cpp include/ros2oigtlGenericConverter.h src/ros2oigtlMesh.cpp include/ros2oigtlDefaultConversions.h src/ros2oigtlDefaultConversions.cpp)
add_executable(oigtl2rosTransformation src/oigtl2rosGenericConverter.cpp include/oigtl2rosGenericConverter.h src/oigtl2rosTransformation.cpp include/ros2oigtlDefaultConversions.h src/ros2oigtlDefaultConversions.cpp)
add_executable(oigtl2rosPivot src/oigtl2rosGenericConverter.cpp include/oigtl2rosGenericConverter.h src/oigtl2rosPivot.cpp include/ros2oigtlDefaultConversions.h src/ros2oigtlDefaultConversions.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(rosopenigtbridge_node rosopenigtbridge_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(ros2openigtlink
   ${catkin_LIBRARIES}
    OpenIGTLink
)

target_link_libraries(ros2oigtlPose
   ${catkin_LIBRARIES}
    OpenIGTLink
)

target_link_libraries(ros2oigtlTransformation
   ${catkin_LIBRARIES}
    OpenIGTLink
)

target_link_libraries(oigtl2rosTransformation
   ${catkin_LIBRARIES}
    OpenIGTLink
)

target_link_libraries(oigtl2rosPivot
   ${catkin_LIBRARIES}
    OpenIGTLink
)



target_link_libraries(ros2oigtlImage
   ${catkin_LIBRARIES}
    OpenIGTLink
)

target_link_libraries(ros2oigtlMesh
   ${catkin_LIBRARIES}
    OpenIGTLink
)



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
 install(TARGETS ros2openigtlink ros2openigtlink
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".git" EXCLUDE
 )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rosopenigtbridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
