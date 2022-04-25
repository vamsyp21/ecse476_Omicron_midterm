cmake_minimum_required(VERSION 2.8.3)
project(arm_motion)

find_package(catkin_simple REQUIRED)
find_package(actionlib_msgs REQUIRED)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

# @Matthew - Added for action lib
# add_action_files(
#  DIRECTORY src/action
#  FILES ArmMotion.action
#)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# Libraries: uncomment the following and edit arguments to create a new library
# cs_add_library(traj_builder src/traj_builder.cpp)   

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
# cs_add_executable(lidar_alarm_mobot src/lidar_alarm_mobot.cpp)
# cs_add_executable(current_state_publisher src/current_state_publisher.cpp)
#cs_add_executable(modal_trajectory_controller src/modal_trajectory_controller.cpp)
# cs_add_executable(des_state_publisher_service src/des_state_publisher_service.cpp)
# cs_add_executable(navigation_coordinator src/navigation_coordinator.cpp)
#cs_add_executable(lin_steering_wrt_odom src/lin_steering_wrt_odom.cpp)
cs_add_executable(arm_motion_action_server src/arm_motion_action_server.cpp)
# Libraries: uncomment the following and edit arguments to create a new library
cs_add_library(baxter_trajectory_streamer src/baxter_trajectory_streamer.cpp)   
cs_add_executable(baxter_playback src/baxter_playfile_jointspace.cpp)

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
#cs_add_executable(trajectory_interpolator_as src/trajectory_interpolator_as.cpp)
cs_add_executable(left_arm_as src/left_arm_as.cpp)
cs_add_executable(rt_arm_as src/rt_arm_as.cpp)
cs_add_executable(pre_pose src/traj_action_client_pre_pose.cpp)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
target_link_libraries(pre_pose baxter_trajectory_streamer)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_libraries(des_state_publisher_service traj_builder)
# target_link_libraries(navigation_coordinator traj_builder)
#target_link_libraries(arm_motion_action_server baxter_trajectory_streamer)

cs_install()
cs_export()