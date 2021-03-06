cmake_minimum_required(VERSION 2.8.3)
project(body_interact_game)

add_compile_options(-std=c++14)

set(DEPEND_PACKAGES
  roscpp
  geometry_msgs
  rviz_visual_tools # for debuging
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
)

find_package(catkin REQUIRED COMPONENTS
  ${DEPEND_PACKAGES}
  message_generation
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
add_message_files(
  FILES
  Score.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES body_interact_game
  CATKIN_DEPENDS ${DEPEND_PACKAGES} message_runtime
  DEPENDS Eigen3
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/body_interact_game.cpp
  src/${PROJECT_NAME}/body_analizer.cpp
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(body_interact_game_manager_node src/body_interact_game_manager_node.cpp src/${PROJECT_NAME}/pose_generator.cpp)
add_dependencies(body_interact_game_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(body_interact_game_manager_node
  ${catkin_LIBRARIES}
)

add_executable(score_viewer_node src/score_viewer_node.cpp src/${PROJECT_NAME}/pose_generator.cpp)
add_dependencies(score_viewer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(score_viewer_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h*"
  PATTERN ".svn" EXCLUDE
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_body_interact_game.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
