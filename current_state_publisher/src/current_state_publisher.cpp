#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <nav_msgs/Odometry.h>
#include <odom_tf/odom_tf.h>



int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "OdomTf_node"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
	//ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("OdomTf_node", 1);
    ros::Publisher pub2 = nh.advertise<nav_msgs::Odometry>("/current_state",10);

    ROS_INFO("main: instantiating an object of type OdomTf");
    OdomTf odomTf(&nh);  //instantiate an OdomTf object and pass in pointer to nodehandle for constructor to use
    OdomTf current_state_subscriber(&nh);

    ROS_INFO:("starting main loop");
    ros::Rate sleep_timer(50.0);
    while (ros::ok()) {
        ros::spinOnce();
        sleep_timer.sleep(); 
    }
    
    return 0;
} 
