//find_table_frame.cpp
// prompts for a pcd file name, reads the file, and displays to rviz on topic "pcd"
// illustrates use of PCL method transformPointCloud(), 
// pcl::PassThrough methods setInputCloud(), setFilterFieldName(), setFilterLimits, filter()
// pcl::io::loadPCDFile() 
// pcl::toROSMsg() for converting PCL pointcloud to ROS message
// voxel-grid filtering: pcl::VoxelGrid,  setInputCloud(), setLeafSize(), filter()
//wsn April 2021

#include <ros/ros.h> 
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
//#include <pcl/ros/conversions.h>
#include <pcl/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2 

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLHeader.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "detect_object/DetectTransformServiceMsg.h"

#include <xform_utils/xform_utils.h>
#include <pcl_utils/pcl_utils.h>  //a local library with some utility fncs
int g_ans;

using namespace std;
ros::NodeHandle* nh_ptr;
bool got_kinect_image = false; //snapshot indicator
bool found_table = false;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud

sensor_msgs::PointCloud2 downsampled_cloud, ransac_cloud ,ros_box_filtered_cloud; //here are ROS-compatible messages
sensor_msgs::PointCloud2 ros_cloud_wrt_table,ros_cloud,ros_planar_cloud;

tf::Transform table_wrt_cam_transform;

void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    if (!got_kinect_image) { // once only, to keep the data stable
        ROS_INFO("got new selected kinect image");
        pcl::fromROSMsg(*cloud, *pclKinect_clr_ptr);
        // change header frame to the one we use
        pclKinect_clr_ptr->header.frame_id = "camera_depth_optical_frame";
        ROS_INFO("image has  %d * %d points", pclKinect_clr_ptr->width, pclKinect_clr_ptr->height);
        got_kinect_image = true;
    }
}

double plane_z_interp(float x, float y, float plane_dist, Eigen::Vector3f normal) {
    double z;
    z = (plane_dist - x * normal(0) - y * normal(1)) / normal(2);
    return z;
}

double createInterpolatedPlaneCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, Eigen::Vector3f plane_normal, double plane_dist, double x_max, double y_max, double x_min, double y_min) {
    //generate points in the corresponding plane:
    //double plane_z_interp(double x, double y, double plane_dist, Eigen::Vector3d normal)

    input_cloud_ptr->points.clear();
    double zPlaneSum = 0;
    int count = 0;
    for (float x = x_min; x < x_max; x += 0.01) {
        for (float y = y_min; y < y_max; y += 0.01) {
            float z = plane_z_interp(x, y, plane_dist, plane_normal);
            zPlaneSum += z;
            count++;
            input_cloud_ptr->points.push_back(pcl::PointXYZ(x, y, z));
        }
    }
    //pcl::toROSMsg(*g_interpolated_plane_fit_cloud, g_interpolated_plane_fit_cloud_ros);
    //g_interpolated_plane_fit_cloud_ros.header.frame_id = "RBIP_frame";
    int n_fitted = input_cloud_ptr->points.size();
    input_cloud_ptr->width = n_fitted;
    input_cloud_ptr->height = 1;

    return zPlaneSum / count;
}

bool detectTableWrtCamCallBack(detect_object::DetectTransformServiceMsgRequest &request, detect_object::DetectTransformServiceMsgResponse &response)
{
    ros::NodeHandle& nh_ref = *nh_ptr;
    ros::Subscriber pointcloud_subscriber = nh_ref.subscribe("/captured_pcl", 1, kinectCB);

    got_kinect_image = false;

    //instantiate pointers to various clouds
    //pclKinect_clr_ptr will contain data from the from the image read in,
    //others are results of processing this original cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ransac_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    pcl::PointCloud<pcl::PointXYZ>::Ptr fitted_plane_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_wrt_table_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    vector<int> indices; //use this to identify points of interest, using indices of these points

    // callback will run and check if there is data
    // spin until obtain a snapshot
    ROS_INFO("waiting for kinect data");
    while (!got_kinect_image) {
        ROS_INFO("waiting...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ransac_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    // if out of this loop -> obtained a cloud and store in pclKinect_clr_ptr
    ROS_INFO("receive data from kinect");

    ROS_INFO("instantiating a pclUtils object");
    PclUtils pclUtils(nh_ptr); //instantiate a PclUtils object--a local library w/ some handy fncs

    //PCD file does not seem to record the reference frame;  set frame_id manually
    pclKinect_clr_ptr->header.frame_id = "camera_depth_optical_frame";

    pcl::toROSMsg(*pclKinect_clr_ptr, ros_cloud); //convert from PCL cloud to ROS message this way

    //use voxel filtering to downsample the original cloud:
    cout << "starting voxel filtering" << endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(pclKinect_clr_ptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ransac_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    vox.setLeafSize(0.02f, 0.02f, 0.02f);
    vox.filter(*downsampled_kinect_ptr);
    cout << "done voxel filtering" << endl;

    cout << "num bytes in original cloud data = " << pclKinect_clr_ptr->points.size() << endl;
    cout << "num bytes in filtered cloud data = " << downsampled_kinect_ptr->points.size() << endl; // ->data.size()<<endl;    
    pcl::toROSMsg(*downsampled_kinect_ptr, downsampled_cloud); //convert to ros message for publication and display

    // test RANSAC
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.005);

    seg.setInputCloud (downsampled_kinect_ptr);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        ROS_ERROR ("Could not estimate a planar model for the given dataset.");
        response.detect_success = false;
        found_table = false;
        return true;
    }   

    // extract from ransac seg
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    // Extract the inliers
    extract.setInputCloud (downsampled_kinect_ptr);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*ransac_kinect_ptr);

    pcl::toROSMsg(*ransac_kinect_ptr, ransac_cloud); //convert to ros message for publication and display

    // box filter
    Eigen::Vector3f box_pt_min, box_pt_max;
    box_pt_min << -0.4, -0.5, 0.2;
    box_pt_max << 0.4, 0.0, 2;


    int nsamps = ransac_kinect_ptr->points.size();
    ROS_INFO("num pts in downsampled cloud: %d", nsamps);

    pclUtils.box_filter(ransac_kinect_ptr, box_pt_min, box_pt_max, indices);

    if (indices.size() == 0) 
    {
        ROS_ERROR ("Found no table within the wanted range");
        response.detect_success = false;
        found_table = false;
        return true;
    }

    pcl::copyPointCloud(*ransac_kinect_ptr, indices, *box_filtered_cloud_ptr); //extract these pts into new cloud
    //the new cloud is a set of points from original cloud, coplanar with selected patch; display the result
    pcl::toROSMsg(*box_filtered_cloud_ptr, ros_box_filtered_cloud); //convert to ros message for publication and display

    //find plane fit to box-filtered cloud
    Eigen::Vector3f plane_normal;
    double plane_dist;

    pclUtils.fit_points_to_plane(box_filtered_cloud_ptr, plane_normal, plane_dist);

    ROS_INFO_STREAM("plane_normal = " << plane_normal.transpose() << "; plane_dist = " << plane_dist << endl);
    createInterpolatedPlaneCloud(fitted_plane_ptr, plane_normal, plane_dist, box_pt_max(0), box_pt_max(1), box_pt_min(0), box_pt_min(1));
    int n_fitted = fitted_plane_ptr->points.size();

    ROS_INFO("fitted plane has %d points", n_fitted);
    pcl::toROSMsg(*fitted_plane_ptr, ros_planar_cloud);
    ros_planar_cloud.header.frame_id = "camera_depth_optical_frame";

    //create a frame on the identified plane:
    Eigen::Vector3f z_plane_wrt_cam, y_plane_wrt_cam, x_plane_wrt_cam;
    Eigen::Matrix3f R_plane_wrt_cam;
    Eigen::Vector3f O_plane_wrt_cam;
    Eigen::Vector3f z_cam_wrt_cam,x_cam_wrt_cam;
    z_cam_wrt_cam << 0, 0, 1;
    x_cam_wrt_cam << 1, 0, 0;
    O_plane_wrt_cam = plane_normal*plane_dist;
    z_plane_wrt_cam = plane_normal;
    
    //use x_plane = -y_camera projected onto identified plane:
    //equiv, construct x_plane = -x_cam x plane_normal
    x_plane_wrt_cam = -x_cam_wrt_cam.cross(plane_normal);
    x_plane_wrt_cam = x_plane_wrt_cam/x_plane_wrt_cam.norm(); //axes must be unit length
    y_plane_wrt_cam = z_plane_wrt_cam.cross(x_plane_wrt_cam);
    
    R_plane_wrt_cam.col(0) = x_plane_wrt_cam;
    R_plane_wrt_cam.col(1) = y_plane_wrt_cam;
    R_plane_wrt_cam.col(2) = z_plane_wrt_cam;
    
    //put the above into an affine transform:
    Eigen::Affine3f affine_plane_wrt_cam, affine_cam_wrt_plane;
    affine_plane_wrt_cam.linear() = R_plane_wrt_cam;
    affine_plane_wrt_cam.translation() = O_plane_wrt_cam;
    cout << "O_plane_wrt_cam: " << O_plane_wrt_cam.transpose() << endl;
    cout << "R_plane_wrt_cam: " << endl << R_plane_wrt_cam << endl;
    affine_cam_wrt_plane = affine_plane_wrt_cam.inverse();

    //convert all points in box_filtered_cloud_ptr into the new table frame, with output in output_cloud_wrt_table_ptr
    pclUtils.transform_cloud(affine_cam_wrt_plane, box_filtered_cloud_ptr, output_cloud_wrt_table_ptr);
    
    //prepare to display the result of the transformed data
    pcl::toROSMsg(*output_cloud_wrt_table_ptr, ros_cloud_wrt_table);
    //pointcloud does not tell us the ROS frame; must install manually
    ros_cloud_wrt_table.header.frame_id = "table_frame";   

    XformUtils xformutils;

    geometry_msgs::Pose table_wrt_cam_pose = xformutils.transformEigenAffine3dToPose(affine_plane_wrt_cam.cast<double>());
    tf::poseMsgToTF(table_wrt_cam_pose, table_wrt_cam_transform);

    response.detect_success = true;
    found_table = true;
    tf::transformStampedTFToMsg(tf::StampedTransform(table_wrt_cam_transform, ros::Time::now(),"camera_depth_optical_frame","table_frame"),response.detect_transform);

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "detect_table_wrt_torso_service"); //node name
    ros::NodeHandle nh;
    nh_ptr = &nh;
    
    // create a service server to detect the table frame
    ros::ServiceServer detect_object_service = nh.advertiseService("detect_table_wrt_cam_service", detectTableWrtCamCallBack);

    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    // ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    // ros::Publisher pubPlane = nh.advertise<sensor_msgs::PointCloud2> ("planar_pts", 1);
    // ros::Publisher pubDnSamp = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_pcd", 1);
    // ros::Publisher pubBoxFilt = nh.advertise<sensor_msgs::PointCloud2> ("box_filtered_pcd", 1);
    ros::Publisher pubTableFrame = nh.advertise<sensor_msgs::PointCloud2> ("table_frame_pts", 1);
    ros::Publisher pubRansac = nh.advertise<sensor_msgs::PointCloud2> ("ransac_cloud",1);
    tf::TransformBroadcaster br;

    ROS_INFO("detect_table_wrt_cam_service is ON");
    ROS_INFO("preparing to check for table_wrt_cam transformation");

    while (ros::ok()) {
        if (found_table == true){
            pubTableFrame.publish(ros_cloud_wrt_table);
            //pubCloud.publish(ros_cloud); // will not need to keep republishing if display setting is persistent
            //pubPlane.publish(ros_planar_cloud); // display the set of points computed to be coplanar w/ selection
            pubRansac.publish(ransac_cloud);
            //pubDnSamp.publish(downsampled_cloud); //can directly publish a pcl::PointCloud2!!
            //pubBoxFilt.publish(ros_box_filtered_cloud);
            br.sendTransform(tf::StampedTransform(table_wrt_cam_transform, ros::Time::now(), "camera_depth_optical_frame", "table_frame"));
        }
        ros::spinOnce(); 
        ros::Duration(0.1).sleep();
    }

    return 0;
}

