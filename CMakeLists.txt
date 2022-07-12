cmake_minimum_required(VERSION 2.8.3)
project(robot_teach_in)

find_package(catkin REQUIRED COMPONENTS roscpp rospy)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp #abb_libegm::abb_libegm 
)

catkin_install_python(PROGRAMS src/communication.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########
## Build ##
###########
find_package(abb_libegm REQUIRED)

include_directories(
 ${catkin_INCLUDE_DIRS}
 #abb_libegm::abb_libegm 
)

#################################
# Build the basic level samples #
#################################

add_executable(${PROJECT_NAME}_node src/robot_teach_in.cpp)
target_link_libraries(${PROJECT_NAME}_node abb_libegm::abb_libegm ${catkin_LIBRARIES})

add_executable(pose_controller_node src/pose_controller.cpp)
target_link_libraries(pose_controller_node abb_libegm::abb_libegm ${catkin_LIBRARIES})

add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node abb_libegm::abb_libegm ${catkin_LIBRARIES})
