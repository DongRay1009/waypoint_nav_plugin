cmake_minimum_required(VERSION 3.0.2)
project(pose_get_plugin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  geometry_msgs
  visualization_msgs
  tf
)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

add_library(${PROJECT_NAME}
  src/goal_tool.cpp
  src/waypoint_saver.cpp
  src/marker_manager.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装图标文件
install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons
)

add_executable(multi_waypoint_nav src/multi_waypoint_nav.cpp)
target_link_libraries(multi_waypoint_nav ${catkin_LIBRARIES})