cmake_minimum_required(VERSION 3.0.2)
project(mobile_robot_simulator)

find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mobile_robot_simulator
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(odom_publisher src/odom_publisher.cpp)
target_link_libraries(odom_publisher ${catkin_LIBRARIES})

add_executable(waypoint_visualizer src/waypoint_visualizer.cpp)
target_link_libraries(waypoint_visualizer ${catkin_LIBRARIES})

add_executable(pure_pursuit src/pure_pursuit.cpp)
target_link_libraries(pure_pursuit ${catkin_LIBRARIES})

add_executable(pure_pursuit_estimated_pose src/pure_pursuit_estimated_pose.cpp)
target_link_libraries(pure_pursuit_estimated_pose ${catkin_LIBRARIES})

add_executable(pure_pursuit_adaptive_velocity src/pure_pursuit_adaptive_velocity.cpp)
target_link_libraries(pure_pursuit_adaptive_velocity ${catkin_LIBRARIES})

add_executable(pure_pursuit_estimated_pose_adaptive_control src/pure_pursuit_estimated_pose_adaptive_control.cpp)
target_link_libraries(pure_pursuit_estimated_pose_adaptive_control ${catkin_LIBRARIES})

add_executable(pure_pursuit_adaptive_velocity_test src/pure_pursuit_adaptive_velocity_test.cpp)
target_link_libraries(pure_pursuit_adaptive_velocity_test ${catkin_LIBRARIES})

add_executable(pure_pursuit_estimated_pose_related_yaw src/pure_pursuit_estimated_pose_related_yaw.cpp)
target_link_libraries(pure_pursuit_estimated_pose_related_yaw ${catkin_LIBRARIES})

add_executable(pure_pursuit_estimated_pose_adaptive_control_related_yaw src/pure_pursuit_estimated_pose_adaptive_control_related_yaw.cpp)
target_link_libraries(pure_pursuit_estimated_pose_adaptive_control_related_yaw ${catkin_LIBRARIES})

add_executable(estimated_pose_pure_pursuit_1028 src/estimated_pose_pure_pursuit_1028.cpp)
target_link_libraries(estimated_pose_pure_pursuit_1028 ${catkin_LIBRARIES})

add_executable(pure_pursuit_crossProblem src/pure_pursuit_crossProblem.cpp)
target_link_libraries(pure_pursuit_crossProblem ${catkin_LIBRARIES})

add_executable(pure_pursuit_crossProblem_estimated src/pure_pursuit_crossProblem_estimated.cpp)
target_link_libraries(pure_pursuit_crossProblem_estimated ${catkin_LIBRARIES})

add_executable(pure_pursuit_test src/pure_pursuit_test.cpp)
target_link_libraries(pure_pursuit_test ${catkin_LIBRARIES})

add_executable(pure_pursuit_1105 src/pure_pursuit_1105.cpp)
target_link_libraries(pure_pursuit_1105 ${catkin_LIBRARIES})

add_executable(pure_pursuit_estimated_1105 src/pure_pursuit_estimated_1105.cpp)
target_link_libraries(pure_pursuit_estimated_1105 ${catkin_LIBRARIES})

add_executable(pure_pursuit_1110 src/pure_pursuit_1110.cpp)
target_link_libraries(pure_pursuit_1110 ${catkin_LIBRARIES})

add_executable(pure_pursuit_estimated_1110 src/pure_pursuit_estimated_1110.cpp)
target_link_libraries(pure_pursuit_estimated_1110 ${catkin_LIBRARIES})