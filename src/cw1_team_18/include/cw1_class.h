#ifndef cw1_CLASS_H_
#define cw1_CLASS_H_

// System includes
#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>
#include <typeinfo>
#include <array>
#include <cmath>
#include <vector>

// ROS message includes
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>

// MoveIt includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>

// TF includes
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>

// Service includes
#include "cw1_world_spawner/Task1Service.h"
#include "cw1_world_spawner/Task2Service.h"
#include "cw1_world_spawner/Task3Service.h"

// Type definitions for PCL
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

/**
 * @brief Main class handling the coursework tasks
 */
class cw1
{
public:
    /**
     * @brief Constructor for the cw1 class
     * @param nh ROS NodeHandle for initialization
     */
    cw1(ros::NodeHandle nh);

    /**
     * @brief Task 1 service callback - Pick and place a cube
     */
    bool t1Callback(cw1_world_spawner::Task1Service::Request &request,
                   cw1_world_spawner::Task1Service::Response &response);

    /**
     * @brief Task 2 service callback - Check basket existence and color
     */
    bool t2Callback(cw1_world_spawner::Task2Service::Request &request,
                   cw1_world_spawner::Task2Service::Response &response);

    /**
     * @brief Task 3 service callback - Sort cubes into corresponding color baskets
     */
    bool t3Callback(cw1_world_spawner::Task3Service::Request &request,
                   cw1_world_spawner::Task3Service::Response &response);

    /**
     * @brief Callback for processing point cloud data for Task 2
     */
    void cloudCallBackOne(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg);

    /**
     * @brief Callback for processing point cloud data for Task 3
     */
    void cloudCallBackTwo(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg_task_3);

private:
    // MoveIt control functions
    /**
     * @brief Move the arm to a target pose
     * @param target_pose Target pose to move to
     * @return Success of the operation
     */
    bool moveArm(geometry_msgs::Pose target_pose);

    /**
     * @brief Control the gripper width
     * @param width Width between fingers
     * @return Success of the operation
     */
    bool moveGripper(float width);

    /**
     * @brief Add a collision object to the planning scene
     */
    void addCollisionObject(std::string object_name, geometry_msgs::Point centre,
                          geometry_msgs::Vector3 dimensions, geometry_msgs::Quaternion orientation);

    /**
     * @brief Remove a collision object from the planning scene
     */
    void removeCollisionObject(std::string object_name);

    /**
     * @brief Remove all collision objects from the planning scene
     */
    void removeAllCollisions();

    // PCL processing functions
    /**
     * @brief Apply voxel grid filtering to a point cloud
     */
    void applyVX(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);

    /**
     * @brief Apply passthrough filtering along X axis
     */
    void applyPT_x(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);

    /**
     * @brief Apply passthrough filtering along Y axis
     */
    void applyPT_y(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);

    /**
     * @brief Estimate surface normals in a point cloud
     */
    void findNormals(PointCPtr &in_cloud_ptr);

    /**
     * @brief Segment planes in a point cloud
     */
    void segPlane(PointCPtr &in_cloud_ptr);

    /**
     * @brief Segment cylindrical shapes in a point cloud
     */
    void segCylind(PointCPtr &in_cloud_ptr);

    /**
     * @brief Find cylinder pose in a point cloud
     */
    void findCylPose(PointCPtr &in_cloud_ptr);

    /**
     * @brief Cluster objects in a point cloud
     */
    void cluster(PointCPtr &in_cloud_ptr);

    /**
     * @brief Publish filtered point cloud for visualization
     */
    void pubFilteredPCMsg(ros::Publisher &pc_pub, PointC &pc);

    /**
     * @brief Publish pose information
     */
    void publishPose(geometry_msgs::PointStamped &cyl_pt_msg);

    /**
     * @brief Callback for target pose updates
     */
    void targetPoseCallback(const geometry_msgs::PointStamped &cyl_pt_msg);

    // Helper functions
    /**
     * @brief Remove duplicate positions from a vector
     */
    std::vector<geometry_msgs::Point> discardRepeatPos(std::vector<geometry_msgs::Point> in_vector);

    /**
     * @brief Add all collision objects for task 3
     */
    void addCollision();

    /**
     * @brief Print cube and basket information for debugging
     */
    void printCubeBasket();

    /**
     * @brief Add floor plate collision object
     */
    void addPlate();

    /**
     * @brief Add cube collision objects
     */
    void addCube(std::vector<geometry_msgs::Point> in_vec, std::string name);

    /**
     * @brief Add basket collision objects
     */
    void addBasket(std::vector<geometry_msgs::Point> in_vec, std::string name);

    /**
     * @brief Pick and place a cube into a basket
     */
    void pickAndPlace(std::vector<geometry_msgs::Point> cube,
                    std::vector<geometry_msgs::Point> basket,
                    std::string cube_name);

    /**
     * @brief Set joint constraints for movement
     */
    void setConstraint();

    // ROS node handling
    ros::NodeHandle nh_;
    ros::ServiceServer t1_service_;
    ros::ServiceServer t2_service_;
    ros::ServiceServer t3_service_;

    // Constants
    std::string base_frame_ = "panda_link0";
    double gripper_open_ = 80e-3;
    double gripper_closed_ = 0.0;
    std::string basket_name = "basket";
    std::string cube_name = "cube";
    std::string plate_name = "plate";

    // MoveIt interfaces
    moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
    moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    // RGB color definitions
    double red[3] = {0.8, 0.1, 0.1};
    double purple[3] = {0.8, 0.1, 0.8};
    double blue[3] = {0.1, 0.1, 0.8};
    uint32_t rgba;

    // ROS publishers
    ros::Publisher g_pub_task_2;
    ros::Publisher g_pub_task_3;
    ros::Publisher pub_cluster;
    ros::Publisher g_pub_cloud;
    ros::Publisher g_pub_pose;

    // PCL filtering parameters
    double g_vg_leaf_sz;
    double g_pt_thrs_min_x, g_pt_thrs_max_x, g_pt_thrs_min_y, g_pt_thrs_max_y;
    double g_k_nn;

    // PCL data structures
    std::string g_input_pc_frame_id_;
    PointCPtr g_cloud_ptr;
    PointCPtr g_cloud_ptr_task_3;
    PointCPtr g_cloud_filtered, g_cloud_filtered2;
    PointCPtr g_cloud_filtered_task_3, g_cloud_filtered2_task_3;
    PointCPtr g_cloud_plane, g_cloud_cylinder;
    pcl::PCLPointCloud2 g_pcl_pc;
    pcl::PCLPointCloud2 g_pcl_pc_task_3;
    sensor_msgs::PointCloud2 g_cloud_filtered_msg;

    // PCL processing objects
    pcl::VoxelGrid<PointT> g_vx;
    pcl::PassThrough<PointT> g_pt;
    pcl::search::KdTree<PointT>::Ptr g_tree_ptr;
    pcl::NormalEstimation<PointT, pcl::Normal> g_ne;
    pcl::PointCloud<pcl::Normal>::Ptr g_cloud_normals, g_cloud_normals2;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> g_seg;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> g_ec;
    pcl::ExtractIndices<PointT> g_extract_pc;
    pcl::ExtractIndices<pcl::Normal> g_extract_normals;
    pcl::PointIndices::Ptr g_inliers_plane;
    pcl::PointIndices::Ptr g_inliers_cylinder;
    pcl::PointIndices::Ptr singal_cluster_ptr;
    pcl::PointCloud<PointT>::Ptr singal_cluster;
    pcl::ModelCoefficients::Ptr g_coeff_plane;
    pcl::ModelCoefficients::Ptr g_coeff_cylinder;
    std::vector<pcl::PointIndices> cluster_indices;

    // TF listener
    tf::TransformListener g_listener_;

    // Message data
    std_msgs::Header cyl_pt_msg_header;
    geometry_msgs::Point cyl_pt_msg_point;
    geometry_msgs::Point task_3_position;
    geometry_msgs::PointStamped g_cyl_pt_msg;

    // MoveIt constraints
    moveit_msgs::Constraints constraints;
    moveit_msgs::JointConstraint joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7;

    // State flags
    bool segCylind_done = false;
    bool cluster_done = false;
    bool moveArm_done = false;
    bool cluster_or_not = false;
    bool task_3_triger = false;
    bool debug_ = false;

    // Data containers for Task 3
    std::vector<geometry_msgs::Point> red_cube, blue_cube, purple_cube;
    std::vector<geometry_msgs::Point> red_basket, blue_basket, purple_basket;
};

#endif // cw1_CLASS_H_