#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <arm_motion/ArmMotionAction.h>
#include <arm_motion/ArmMotionFeedback.h>
#include <arm_motion/ArmMotionResult.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <ros/ros.h>
#include <stdlib.h>     /* getenv */
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>

#include <std_msgs/Int32.h>
#include <arm_motion/trajAction.h>

#define VECTOR_DIM 7 // e.g., a 7-dof vector

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

using namespace std;

typedef vector <double> record_t;
typedef vector <record_t> data_t;

//vector.pushback()
//data_t.pushback(record_t)
//-1, 1, 0, 0, 0, 0, 0, 0.0 -1, 1, 3, 0, 0, 0, 0, 2.0 -1, 0, 3, 0, 0, 0, 0, 4.0 -1, 1, 3, 1.87, 0, 0, 0, 6.0
//-1, -0.11, 3, 1.87, -1.3,      2.0, 0, 8.0
//-0.90,    -0.11,      2.06,      1.87,     -1.3, 2.0, 0, 10.0

//split(" ")


// Global current state and current pose (updated by current state subscriber callback)
nav_msgs::Odometry current_state; // Current State (odom)
geometry_msgs::PoseStamped current_pose; // Current Position

class ArmMotionAction {
    private:
        ros::NodeHandle _node_handle; // Node handle

        // *** Action Control (Interface/Master) ***
        // Action Server (Goal, Feedback, Result)
        std::string _action_name; // Action Server's Name
        actionlib::SimpleActionServer<arm_motion::ArmMotionAction> _action_server; // Action Server
        arm_motion::ArmMotionFeedback _feedback; // Feedback
        arm_motion::ArmMotionResult _result; // Result


        // Booleans for tracking success and completion
        bool g_got_good_traj_right = false;
        bool g_got_good_traj_left = false;
        bool g_right_arm_done = false;
        bool g_left_arm_done = false;

        void leftArmDoneCb(const actionlib::SimpleClientGoalState& state, const baxter_trajectory_streamer::trajResultConstPtr & result) {
            ROS_INFO(" leftArmDoneCb: server responded with state [%s]", state.toString().c_str());
            ROS_INFO("got return val = %d", result->return_val);
            g_left_arm_done = true;
        }

        int read_traj(string jsp_data, trajectory_msgs::JointTrajectory &des_trajectory){
            data_t data = jsp_string_to_data_t(jsp_data);

            //data is valid; pack it up as a trajectory and ship it
            //trajectory_msgs::JointTrajectory des_trajectory; // an empty trajectory 
            des_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
            des_trajectory.joint_names.clear(); //could put joint names in...but I assume a fixed order and fixed size, so this is unnecessary
            des_trajectory.header.stamp = ros::Time::now();

            trajectory_msgs::JointTrajectoryPoint trajectory_point; //,trajectory_point2; 
            trajectory_point.positions.resize(7);
            double t_arrival;
            for (unsigned n = 0; n < data.size(); n++) {
                // pack trajectory points, one at a time:
                for (int i = 0; i < 7; i++) {
                    trajectory_point.positions[i] = data[n][i];
                }
                t_arrival = data[n][7];
                trajectory_point.time_from_start = ros::Duration(t_arrival);
                des_trajectory.points.push_back(trajectory_point);
            }
            return 0;
        }

        // To be initialized in the constructor
        std::string g_path_to_playfiles;
        Baxter_traj_streamer *g_baxter_traj_streamer_ptr;
        actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> *g_left_arm_action_client_ptr;
        actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> *g_right_arm_action_client_ptr;

    public:

        ArmMotionAction(std::string name) : _action_server(_node_handle, name, boost::bind(&ArmMotionAction::executeActionCallback, this, _1), false), _action_name(name)
        {
            _action_server.start(); // Start Action Server
        }

        ~ArmMotionAction(void){}

        // https://github.com/wsnewman/learning_ros_noetic/blob/991c494ffcf29d30e6c40db0f49d88e7bf59513d/Part_5/baxter/baxter_playfile_nodes/src/baxter_playfile_service.cpp
        void executeActionCallback(const arm_motion::ArmMotionGoalConstPtr &goal)
        {
            ros::Rate r(1);
            bool success = false;

            /* _feedback.distance_to_goal = distance_to_goal(current_pose, goal_pos->position_x, goal_pos->position_y);

            // publish info to the console for the user
            ROS_INFO("%s: Executing, locomotion to position: (%f,%f) -> distance to goal_pos (%f)", _action_name.c_str(), goal_pos->position_x, goal_pos->position_y, _feedback.distance_to_goal);

            // start executing the action
            for(int i=1; i<=DEFAULT_RETRY_MOVES; i++)
            {

                // check that preempt has not been requested by the client
                if (_action_server.isPreemptRequested() || !ros::ok())
                {
                    ROS_INFO("%s: Preempted", _action_name.c_str());
                    // set the action state to preempted
                    _action_server.setPreempted();
                    success = false;
                    break;
                }

                // If we have yet to be successful - calculate distance to the goal_pos
                if(success == false)
                {
                    _feedback.distance_to_goal = distance_to_goal(current_pose, goal_pos->position_x, goal_pos->position_y);
                
                    _action_server.publishFeedback(_feedback); // publish the feedback

                    success = move2coord(goal_pos->position_x, goal_pos->position_y);

                    _feedback.distance_to_goal = distance_to_goal(current_pose, goal_pos->position_x, goal_pos->position_y);

                    _action_server.publishFeedback(_feedback); // publish the feedback
                }
            }

            // If successful, set the action server's message-result and print information for the user
            if(success)
            {
                _result.success = success;
                ROS_INFO("%s: Succeeded", _action_name.c_str());
                // set the action state to succeeded
                _action_server.setSucceeded(_result);
            } */
            ROS_INFO("Execute Action Motion Callback activated");
            int playfile_code = goal->playfile_code;

            Eigen::VectorXd q_left_state, q_left_firstpoint // q_left_state, q_left_firstpoint;
            q_left_firstpoint.resize(7);

            Vectorq7x1 q_vec_left_arm
            baxter_trajectory_streamer::trajGoal goal_left

            std::vector<Eigen::VectorXd> des_path_left

            trajectory_msgs::JointTrajectory des_trajectory_left // objects to hold trajectories
            trajectory_msgs::JointTrajectory approach_trajectory_left // objects to hold trajectories    
            trajectory_msgs::JointTrajectoryPoint trajectory_point0;

            //get left arm angles; start motion from here
            q_vec_left_arm = g_baxter_traj_streamer_ptr->get_qvec_left_arm();
            q_left_state = q_vec_left_arm; // start from here;
 
            des_path_left.clear();
            des_path_left.push_back(q_left_state);

            g_got_good_traj_left = false;
            switch (playfile_code) {
                case baxter_playfile_nodes::playfileSrvRequest::PRE_POSE:
                    ROS_INFO("case PRE_POSE:  ");
                    //construct the full path to the filename:
                    // Read data from string
                    jsp_data_string = [];
                    
                    //test if this file can be opened and parsed:
                    if (0 == read_traj(jsp_data_string, des_trajectory_left)) {
                        ROS_INFO("read left-arm file OK");
                        g_got_good_traj_left = true;
                    }
                case baxter_playfile_nodes::playfileSrvRequest::CMD_ANY_JSP_TRAJ:
                    ROS_INFO("case CMD_ANY_JSP_TRAJ:  ");
                    jsp_data_string = [];

                    ROS_INFO("case 1:  baxter_r_arm_traj.jsp and baxter_l_arm_traj.jsp");
                    if (0 == read_traj_file(jsp_data_string, des_trajectory_left)) {
                        ROS_INFO("read left-arm file OK");
                        g_got_good_traj_left = true;
                    }
                    break;
                default:
                    ROS_INFO("unknown case");
                    response.return_code = baxter_playfile_nodes::playfileSrvResponse::UNKNOWN_CASE;
                    return true;
                    break;
            }

            // now have current arm poses and desired trajectories; splice in a motion from current arm pose
            // to first point of desired trajectory;
            if (g_got_good_traj_left) {
                //get first pt of traj: q_left_firstpoint
                trajectory_point0 = des_trajectory_left.points[0];
                for (int i = 0; i < 7; i++) { //copy from traj point to Eigen-type vector
                    q_left_firstpoint[i] = trajectory_point0.positions[i];
                }
                //add this pt to path from current pose:
                des_path_left.push_back(q_left_firstpoint);
                //use traj stuffer to find build trajectory from current pose to first point of recorded traj
                g_baxter_traj_streamer_ptr->stuff_trajectory_left_arm(des_path_left, approach_trajectory_left);
            }

            //command the approach trajectory/trajectories and wait for conclusion   
            g_left_arm_done = true;

            if (g_got_good_traj_left) {
                goal_left.trajectory = approach_trajectory_left;
                g_left_arm_done = false; //reset status trigger, so can check when done
                g_left_arm_action_client_ptr->sendGoal(goal_left, &leftArmDoneCb); // we could also name additional callback functions here, if desired
                //    left_arm_action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
            }

            while (!g_left_arm_done) {
                ROS_INFO("waiting on arm server(s) to approach start of traj");
                ros::Duration(0.5).sleep();
                ros::spinOnce();
            }

            // now send the desired trajectory from file, if there are any points left to execute
            if (g_got_good_traj_left && (des_trajectory_left.points.size() > 1)) {
                goal_left.trajectory = des_trajectory_left;
                g_left_arm_done = false; //reset status trigger, so can check when done
                g_left_arm_action_client_ptr->sendGoal(goal_left, &leftArmDoneCb); // we could also name additional callback functions here, if desired
                //    left_arm_action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
            }

            while (!g_left_arm_done) {
                ROS_INFO("waiting on arm server(s) to execute playfile(s)");
                ros::Duration(0.5).sleep();
                ros::spinOnce();
            }
            return true;
        }

        }

        bool srv_callback(baxter_playfile_nodes::playfileSrvRequest& request, baxter_playfile_nodes::playfileSrvResponse& response) {
            ROS_INFO("srv_callback activated");
            // std::string fname_full_path;
            // int playfile_code = request.playfile_code;

            Eigen::VectorXd q_left_state, q_left_firstpoint // q_left_state, q_left_firstpoint;
            q_left_firstpoint.resize(7);
            //q_left_firstpoint.resize(7);

            Vectorq7x1 q_vec_left_arm //q_vec_left_arm;
            baxter_trajectory_streamer::trajGoal goal_left //goal_left;

            std::vector<Eigen::VectorXd> des_path_left //des_path_left;

            trajectory_msgs::JointTrajectory des_trajectory_left //des_trajectory_left; // objects to hold trajectories
            trajectory_msgs::JointTrajectory approach_trajectory_left //approach_trajectory_left; // objects to hold trajectories    
            trajectory_msgs::JointTrajectoryPoint trajectory_point0;

            //get right and left arm angles; start motion from here
            q_vec_left_arm = g_baxter_traj_streamer_ptr->get_qvec_left_arm();
            q_left_state = q_vec_left_arm; // start from here;
            //q_vec_left_arm = g_baxter_traj_streamer_ptr->get_qvec_left_arm();
            //q_left_state = q_vec_left_arm; // start from here;  
            des_path_left.clear();
            des_path_lefty.push_back(q_left_state);
            //des_path_left.clear();
            //des_path_left.push_back(q_left_state);


            g_got_good_traj_left = false;
            //g_got_good_traj_left = false;
            switch (playfile_code) {
                case baxter_playfile_nodes::playfileSrvRequest::PRE_POSE:
                    ROS_INFO("case PRE_POSE:  ");
                    //construct the full path to the filename:
                    // Read data from string
                    jsp_data_string = [];
                    
                    //test if this file can be opened and parsed:
                    if (0 == read_traj(jsp_data_string, des_trajectory_left)) {
                        ROS_INFO("read left-arm file OK");
                        g_got_good_traj_left = true;
                    }
                case baxter_playfile_nodes::playfileSrvRequest::CMD_ANY_JSP_TRAJ:
                    ROS_INFO("case CMD_ANY_JSP_TRAJ:  ");
                    jsp_data_string = [];

                    ROS_INFO("case 1:  baxter_r_arm_traj.jsp and baxter_l_arm_traj.jsp");
                    if (0 == read_traj_file(jsp_data_string, des_trajectory_left)) {
                        ROS_INFO("read left-arm file OK");
                        g_got_good_traj_left = true;
                    }
                    break;
                default:
                    ROS_INFO("unknown case");
                    response.return_code = baxter_playfile_nodes::playfileSrvResponse::UNKNOWN_CASE;
                    return true;
                    break;
            }

            //now have current arm poses and desired trajectories; splice in a motion from current arm pose
            // to first point of desired trajectory;
            if (g_got_good_traj_left) {
                //get first pt of traj: q_left_firstpoint
                trajectory_point0 = des_trajectory_left.points[0];
                for (int i = 0; i < 7; i++) { //copy from traj point to Eigen-type vector
                    q_left_firstpoint[i] = trajectory_point0.positions[i];
                }
                //add this pt to path from current pose:
                des_path_left.push_back(q_left_firstpoint);
                //use traj stuffer to find build trajectory from current pose to first point of recorded traj
                g_baxter_traj_streamer_ptr->stuff_trajectory_left_arm(des_path_left, approach_trajectory_left);
            }

            //command the approach trajectory/trajectories and wait for conclusion   
            g_left_arm_done = true;

            if (g_got_good_traj_left) {
                goal_left.trajectory = approach_trajectory_left;
                g_left_arm_done = false; //reset status trigger, so can check when done
                g_left_arm_action_client_ptr->sendGoal(goal_left, &leftArmDoneCb); // we could also name additional callback functions here, if desired
                //    left_arm_action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
            }

            while (!g_left_arm_done) {
                ROS_INFO("waiting on arm server(s) to approach start of traj");
                ros::Duration(0.5).sleep();
                ros::spinOnce();
            }

            // now send the desired trajectory from file, if there are any points left to execute
            if (g_got_good_traj_left && (des_trajectory_left.points.size() > 1)) {
                goal_left.trajectory = des_trajectory_left;
                g_left_arm_done = false; //reset status trigger, so can check when done
                g_left_arm_action_client_ptr->sendGoal(goal_left, &leftArmDoneCb); // we could also name additional callback functions here, if desired
                //    left_arm_action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
            }

            while (!g_left_arm_done) {
                ROS_INFO("waiting on arm server(s) to execute playfile(s)");
                ros::Duration(0.5).sleep();
                ros::spinOnce();
            }
            return true;
        }
};

//Action Server
int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_motion_action_server");
    ArmMotionAction ArmMotionAction("arm_motion_action_server");
    ros::spin();
    
    return 0;
}