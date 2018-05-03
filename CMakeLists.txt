cmake_minimum_required(VERSION 2.8.3)
project(humanitude)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  OpenCV
  cv_bridge
  gflags
  image_transport
  roscpp
  std_msgs
  message_generation
  tf

  sensor_msgs
  )

find_package(OpenCV REQUIRED)

set(caffe_DIR /home/baard/caffe)
set(caffe_INCLUDE_DIRS ${caffe_DIR}/distribute/include)
set(caffe_LINK_LIBRARY ${caffe_DIR}/distribute/lib/libcaffe.so)

set(openpose_DIR /home/baard/openpose)
set(openpose_INCLUDE_DIRS ${openpose_DIR}/include)
set(openpose_LINK_LIBRARY ${openpose_DIR}/build/lib/libopenpose.so)

set(cuda_DIR /usr/local/cuda)
set(cuda_INCLUDE_DIRS ${cuda_DIR}/include)
set(cuda_LINK_LIBRARY ${cuda_DIR}/lib64)

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES humanitude
#  CATKIN_DEPENDS OpenCV cv_bridge gflags image_transport roscpp roscpp std_msgs tf
#  DEPENDS system_lib

CATKIN_DEPENDS roscpp message_runtime cv_bridge

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
${catkin_INCLUDE_DIRS}
${caffe_INCLUDE_DIRS}
${openpose_INCLUDE_DIRS}
${cuda_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/humanitude.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/humanitude_node.cpp)

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

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_humanitude.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(humanitude src/humanitude_project.cpp)
add_dependencies(humanitude ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} Person.cpp pose_estimator.cpp camera_subscriber.cpp camera_single_subscriber.h)
target_link_libraries(humanitude ${catkin_LIBRARIES}
  ${openpose_LINK_LIBRARY}
  ${caffe_LINK_LIBRARY}
  ${OpenCV_LIBS}
  )
