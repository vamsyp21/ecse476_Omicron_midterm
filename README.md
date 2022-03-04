# Mobot_Path_Execution_Omicron - Midterm Assignment

## Extended Work From PS4 Mobot Path Execution Assignment
  - Problem Set 4 Assignment for Mobile Robotics
  - https://github.com/mab405/Mobot_Path_Execution_Omicron.git
  
## Reference Code
   - Code from Professor Newman's Repo was used as refernce
     - NOETIC CODE
       - https://github.com/wsnewman/learning_ros_noetic.git
     - MELODIC CODE
       - https://github.com/wsnewman/learning_ros.git
## LAUNCH
   - Can rode code on Jinx OR in Gazebo Simulator Environment
     - GAZEBO: roslaunch mobot_urdf mobot_in_pen.launch
   - Created Launch file to run the following nodes (nodes we altered):
     - des_pub_state_service des_pub_state_service
     - lidar_alarm_mobot lidar_alarm_mobot
     - modal_trajectory_controller lin_steering_wrt_amcl_and_odom
     - modal_trajectory_controller lin_steering_wrt_odom
     - modal_trajectory_controller modal_trajectory_controller
     - odom_tf odom_tf_demo
   - We then manually run: rosurn navigation_coordinator navigation_coordinator
     - This code uses the above information and processes to command the robot to move to a specified point
     - In lab we manually moved the robot through a series of movements and recorded the information on the robots position/rotation
     - We then altered our code to manually move our robot to these locations/poses

### des_pub_state_service
Publishes the current robot state
Also subscribes/publishes to current_state

### lidar_alarm_mobot
Publihses True and False if robot senses object within set distance of lidar

### modal_trajectory_controller
   - Controls the steering of the mobot. Has added 'cases' created from default file that for the following
   - lin_steering_wrt_odom.cpp
     - STOP
     - BACKUP
     - FORWARD
     - HALT
     - BACKUP
### odom_tf
Publishes information regarding the robots odometry for use in other files
    

## Structure
1. For Conveience was structured into a series of 'catkin_create_pkg' package files
   - Conveient for future assignments
2. Each file contains:
   - src
     - code
   - CMAKE
   - Project.xml
