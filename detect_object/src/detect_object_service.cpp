// detect_object
// prompts for a pcd file name, reads the file, and displays to rviz on topic "pcd"
// uses published transform for table frame
// extracts pointcloud points above the table surface
// return a coordinate of the object pose (for now can only detect 1 single object)

#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLHeader.h>

#include <pcl/filters/statistical_outlier_removal.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_utils/pcl_utils.h> //a local library with some utility fncs
#include <xform_utils/xform_utils.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/SVD>

#include "detect_object/DetectTransformServiceMsg.h"


using namespace std;
ros::NodeHandle* nh_ptr;
bool got_kinect_image = false; //snapshot indicator
bool found_block = false;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
sensor_msgs::PointCloud2 ros_cloud_wrt_table, filtered_ros_cloud_wrt_table, ros_pts_above_table, ros_cloud_orig, ros_pts_of_object_on_table;   // for debugging

tf::Transform block_transform;    // transformation of the block wrt the torso

void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    if (!got_kinect_image) { // once only, to keep the data stable
        ROS_INFO("got new selected kinect image");
        pcl::fromROSMsg(*cloud, *pclKinect_clr_ptr);
        // change header frame to the one we use
        pclKinect_clr_ptr->header.frame_id = "camera_rgb_optical_frame"; // originally was depth one, not rgb
        ROS_INFO("image has  %d * %d points", pclKinect_clr_ptr->width, pclKinect_clr_ptr->height);
        got_kinect_image = true;
    }
}

void find_indices_of_plane_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, vector<int> &indices)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object
    pass.setInputCloud(input_cloud_ptr);     //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z");            // we will "filter" based on points that lie within some range of z-value
    pass.setFilterLimits(0.02, 0.05);        //retain points with z values between these limits
    pass.filter(indices);                    //  this will return the indices of the points in given cloud that pass our test
    cout << "number of points passing the filter = " << indices.size() << endl;
    //This fnc populates the reference arg "indices", so the calling fnc gets the list of interesting points
}

Eigen::Affine3f get_table_frame_wrt_camera()
{
    bool tferr = true;
    int ntries = 0;
    XformUtils xformUtils;
    tf::TransformListener tfListener;
    tf::StampedTransform table_frame_wrt_cam_stf;

    Eigen::Affine3f affine_table_wrt_camera;
    while (tferr)
    {
        tferr = false;
        try
        {
            tfListener.lookupTransform("camera_rgb_optical_frame", "table_frame", ros::Time(0), table_frame_wrt_cam_stf); // was rgb
        }
        catch (tf::TransformException &exception)
        {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
            ntries++;
            if (ntries > 5)
            {
                ROS_WARN("did you launch robot's table_frame_wrt_cam.launch?");
                ros::Duration(1.0).sleep();
            }
        }
    }
    ROS_INFO("tf is good for table w/rt camera");
    xformUtils.printStampedTf(table_frame_wrt_cam_stf);

    tf::Transform table_frame_wrt_cam_tf = xformUtils.get_tf_from_stamped_tf(table_frame_wrt_cam_stf);
    affine_table_wrt_camera = xformUtils.transformTFToAffine3f(table_frame_wrt_cam_tf);
    //ROS_INFO("affine: ");
    //xformUtils.printAffine(affine_table_wrt_camera);
    return affine_table_wrt_camera;
}


// A helper function to calculate the centroid of a pointcloud
bool find_centroid_vec(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, Eigen::Vector3f &C_vec)
{
    return true;
}

// A helper function to calculate the rotation matrix of a block using it's upper surface
//! ASSUMPTION: the block is layed flat on the table. (longest axis parallel to the table)
bool find_object_rotation_mat(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, Eigen::Vector3f &C_vec, Eigen::Matrix3f &R_mat)
{
    ROS_INFO("Finding rotation matrix");
    // number of point of the upper surface of the plot
    int npts = input_cloud_ptr->points.size();
    
    // stop n return if there is no data in input_cloud_ptr
    if (npts == 0) 
    {
        ROS_ERROR("There is no block to find rotation");
        return false;
    }

    // define a matrix storing x,y coord of cloud centered about centroid.
    Eigen::MatrixXf pcl_cen(2,npts);
    
    // populate the matrix with x,y data from the cloud
    // center the pointcloud around its centroid
    for (int i = 0; i < npts; i++)
    {
        pcl_cen(0,i) = input_cloud_ptr->points[i].x - C_vec.x();
        pcl_cen(1,i) = input_cloud_ptr->points[i].y - C_vec.y();
    }

    // scatter matrix
    Eigen::Matrix2f S = pcl_cen*(pcl_cen.transpose());

    // SVD to find major axis:
    JacobiSVD<Matrix2f> svd(S, ComputeFullU);
    
    Eigen::Vector2f x_vec, y_vec;
    cout << "U = " << svd.matrixU() << endl;
    cout << "sig1 = " << svd.singularValues()[0] << " | " << "sig2 = " << svd.singularValues()[1] << endl;

    if (svd.singularValues()[0] >= svd.singularValues()[1])
    {
        x_vec = svd.matrixU().col(0);
        y_vec = svd.matrixU().col(1);
    }
    else
    {
        x_vec = svd.matrixU().col(1);
        y_vec = svd.matrixU().col(0);
    }

    // Eigen::Matrix2f U = svd.matrixU();
    R_mat = Matrix3f::Zero();
    R_mat.block<2,1>(0,0) = x_vec;
    R_mat.block<2,1>(0,1) = y_vec;
    R_mat(2,2) = 1;

    cout << "R_mat raw" << endl;
    cout << R_mat << endl;

    // Due to unknown direction of the eigenvector, we just want to ensure that the z of the block is always upward wrt table.
    if (R_mat.determinant() < 0)    R_mat.col(1) = -R_mat.col(1); // swap so x pointing to the left of the robot

    cout << "R_mat adjusted" << endl;
    cout << R_mat << endl;

    return true;
}

// This function is to update the current variable block_transform.
bool detectObjectCallBack(detect_object::DetectTransformServiceMsgRequest &request, detect_object::DetectTransformServiceMsgResponse &response)
{
    ros::NodeHandle& nh_ref = *nh_ptr;
    // subscribe to the pointcloud2 topic
    ros::Subscriber pointcloud_subscriber = nh_ref.subscribe("/captured_pcl", 1, kinectCB);

    // reset the flag to check for kinect image
    got_kinect_image = false;

    // callback will run and check if there is data
    // spin until obtain a snapshot
    ROS_INFO("waiting for kinect data");
    while (!got_kinect_image) {
        ROS_INFO("waiting...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    // if out of this loop -> obtained a cloud and store in pclKinect_clr_ptr
    ROS_INFO("receive data from kinect");

    ROS_INFO("instantiating a pclUtils object");
    PclUtils pclUtils(nh_ptr);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_wrt_table_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_output_cloud_wrt_table_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts_above_table_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts_of_object_on_table_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::toROSMsg(*pclKinect_clr_ptr, ros_cloud_orig); //convert from PCL cloud to ROS message this way
    ros_cloud_orig.header.frame_id = "camera_rgb_optical_frame";

    //find the transform of table w/rt camera and convert to an affine
    Eigen::Affine3f affine_table_wrt_cam, affine_cam_wrt_table;
    affine_table_wrt_cam = get_table_frame_wrt_camera();
    affine_cam_wrt_table = affine_table_wrt_cam.inverse();

    //* Transform the cloud to be w.r.t. a set frame.
    //* For this case the set frame is the table_frame.
    pclUtils.transform_cloud(affine_cam_wrt_table, pclKinect_clr_ptr, output_cloud_wrt_table_ptr);
    pcl::toROSMsg(*output_cloud_wrt_table_ptr, ros_cloud_wrt_table);
    ros_cloud_wrt_table.header.frame_id = "table_frame";

    // filter outlier using statisticaloutlier removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(output_cloud_wrt_table_ptr);
    sor.setMeanK(100);
    sor.setStddevMulThresh(0.7);
    sor.filter(*filtered_output_cloud_wrt_table_ptr);
    pcl::toROSMsg(*filtered_output_cloud_wrt_table_ptr, filtered_ros_cloud_wrt_table);
    filtered_ros_cloud_wrt_table.header.frame_id = "table_frame";

    vector<int> indices;
    find_indices_of_plane_from_patch(filtered_output_cloud_wrt_table_ptr, indices);
    pcl::copyPointCloud(*filtered_output_cloud_wrt_table_ptr, indices, *pts_above_table_ptr); //extract these pts into new cloud
    pcl::toROSMsg(*pts_above_table_ptr, ros_pts_above_table);
    ros_pts_above_table.header.frame_id = "table_frame";

    // vector<int> indices;
    // find_indices_of_plane_from_patch(output_cloud_wrt_table_ptr, indices);
    // pcl::copyPointCloud(*output_cloud_wrt_table_ptr, indices, *pts_above_table_ptr); //extract these pts into new cloud
    // pcl::toROSMsg(*pts_above_table_ptr, ros_pts_above_table);
    // ros_pts_above_table.header.frame_id = "table_frame";

    //! Is this defined in the table_frame?
    Eigen::Vector3f box_pt_min, box_pt_max;
    box_pt_min << -1, -1, 0.025;
    box_pt_max << 1, 0.2, 0.1;
    
    //! Filter out outliers
    pclUtils.box_filter(pts_above_table_ptr, box_pt_min, box_pt_max, indices);
    pcl::copyPointCloud(*pts_above_table_ptr, indices, *pts_of_object_on_table_ptr); //extract these pts into new cloud
    pcl::toROSMsg(*pts_of_object_on_table_ptr, ros_pts_of_object_on_table);           //convert to ros message for publication and display
    ros_pts_of_object_on_table.header.frame_id = "table_frame";

    // calculating the coordinate of the blocks.
    int npts_block_size = pts_of_object_on_table_ptr->points.size();

    float block_x = 0;
    float block_y = 0;
    float block_z = 0;
    for (int i = 0; i < npts_block_size; i++)
    {
        block_x += pts_of_object_on_table_ptr->points[i].x;
        block_y += pts_of_object_on_table_ptr->points[i].y;
    }

    block_x = block_x/npts_block_size;
    block_y = block_y/npts_block_size;

    Eigen::Vector3f centroid(block_x, block_y, 0);
    Eigen::Matrix3f R_mat;
    find_object_rotation_mat(pts_of_object_on_table_ptr, centroid, R_mat);

    cout << "R_mat_det = " << R_mat.determinant() << endl;

    Eigen::Quaternionf q(R_mat);

    cout << "quat = " << q.x() << " , " << q.y() << " , " << q.z() << " , " << q.w() << endl;

    if (isnan(block_x) || isnan(block_y) || npts_block_size == 0)
    {
        ROS_INFO("Could not found any block on the table");
        response.detect_success = false;
        found_block = false;
    }
    else
    {
        ROS_INFO("Found a block on the table");
        block_transform.setOrigin(tf::Vector3(block_x, block_y, block_z));
        block_transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        response.detect_success = true;
        found_block = true;
        tf::transformStampedTFToMsg(tf::StampedTransform(block_transform, ros::Time::now(),"table_frame","block_frame"),response.detect_transform);
    }
    
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_object_service"); //node name
    ros::NodeHandle nh;
    nh_ptr = &nh;

    // create a service server that return the location of an object (if there exists one)
    ros::ServiceServer detect_object_service = nh.advertiseService("detect_object_service", detectObjectCallBack);
    ros::Publisher object_pcl = nh.advertise<sensor_msgs::PointCloud2>("object_pcl",1);
    ros::Publisher fitted_pcl = nh.advertise<sensor_msgs::PointCloud2>("fitted_pcl",1);
    //ros::Publisher orig_pcl = nh.advertise<sensor_msgs::PointCloud2>("orig_pcl",1);

    ROS_INFO("detect_object_service is ON");
    ROS_INFO("preparing to check for block location on table");
        // check the call back
    
    tf::TransformBroadcaster br;
    //ros_cloud_wrt_table.header.frame_id = "table_frame";
    while (ros::ok())
    {   
        if (found_block == true)
        {   
            //orig_pcl.publish(ros_cloud_orig);
            fitted_pcl.publish(ros_cloud_wrt_table);
            object_pcl.publish(ros_pts_of_object_on_table);
            br.sendTransform(tf::StampedTransform(block_transform, ros::Time::now(), "table_frame", "block_frame"));
        }
        ros::spinOnce(); //pclUtils needs some spin cycles to invoke callbacks for new selected points
        ros::Duration(0.1).sleep();
    }

    return 0;
}

