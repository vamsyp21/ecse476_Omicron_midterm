#include <ros/ros.h> 
#include <stdlib.h>
#include <math.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>

ros::NodeHandle* nh_ptr;
sensor_msgs::PointCloud2 captured_pcl;
bool got_kinect_image = false;

void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    if (!got_kinect_image) { // once only, to keep the data stable
        ROS_INFO("got new selected kinect image");
        captured_pcl = *cloud;
        // change header frame to the one we use
        // captured_pcl.header.frame_id = "camera_rgb_optical_frame";
        got_kinect_image = true;
    }
}

bool capturePclCB(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response )
{
    // reset the flag to check for kinect image
    got_kinect_image = false;
    ros::NodeHandle& nh_ref = *nh_ptr;

    //! for actual robot, the topic isn't "/pcd"
    //! use "/camera/depth_registered/points" instead
    // callback will run and check if there is data
    ros::Subscriber pointcloud_subscriber = nh_ref.subscribe("/camera/depth_registered/points", 1, kinectCB);
    
    // spin until obtain a snapshot
    ROS_INFO("waiting for kinect data");
    while (!got_kinect_image) {
        ROS_INFO("waiting...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("captured pcl successfully");
    return true;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "capture_pcl_service"); //node name
    ros::NodeHandle nh;
    nh_ptr = &nh;
    
    // create a service server to detect the table frame
    ros::ServiceServer capture_pcl_service = nh.advertiseService("capture_pcl_service", capturePclCB);

    ros::Publisher captured_pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("captured_pcl",1);

    ROS_INFO("capture_pcl_service is ON");
    ROS_INFO("prepared to capture and publish pcl snapshot");

    while (ros::ok()) {
        if (got_kinect_image == true){
            captured_pcl_pub.publish(captured_pcl);
        }
        ros::spinOnce(); 
        ros::Duration(0.1).sleep();
    }

    return 0;
}

