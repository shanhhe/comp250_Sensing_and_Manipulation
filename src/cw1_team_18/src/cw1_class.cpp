#include <cw1_class.h>

// Constructor
cw1::cw1(ros::NodeHandle nh):
  g_cloud_ptr(new PointC),
  g_cloud_ptr_task_3(new PointC),
  g_cloud_filtered(new PointC),
  g_cloud_filtered2(new PointC),
  g_cloud_filtered_task_3(new PointC),
  g_cloud_filtered2_task_3(new PointC),
  g_cloud_plane(new PointC),
  g_cloud_cylinder(new PointC),
  g_tree_ptr(new pcl::search::KdTree<PointT>),
  singal_cluster_ptr(new pcl::PointIndices),
  singal_cluster(new pcl::PointCloud<PointT>),
  g_cloud_normals(new pcl::PointCloud<pcl::Normal>),
  g_cloud_normals2(new pcl::PointCloud<pcl::Normal>),
  g_inliers_plane(new pcl::PointIndices),
  g_inliers_cylinder(new pcl::PointIndices),
  g_coeff_plane(new pcl::ModelCoefficients),
  g_coeff_cylinder(new pcl::ModelCoefficients)
{
  nh_ = nh;

  // Advertise services for coursework tasks
  t1_service_ = nh_.advertiseService("/task1_start", &cw1::t1Callback, this);
  t2_service_ = nh_.advertiseService("/task2_start", &cw1::t2Callback, this);
  t3_service_ = nh_.advertiseService("/task3_start", &cw1::t3Callback, this);

  // Subscribe to target pose
  ros::Subscriber sub_target_pose = nh_.subscribe("/target_pos", 10, &cw1::targetPoseCallback, this);

  // Define publishers
  g_pub_task_2 = nh_.advertise<sensor_msgs::PointCloud2>("filtered_task_2", 1, true);
  g_pub_task_3 = nh_.advertise<sensor_msgs::PointCloud2>("filtered_task_3", 1, true);
  g_pub_pose = nh_.advertise<geometry_msgs::PointStamped>("target_pos", 1, true);
  pub_cluster = nh_.advertise<sensor_msgs::PointCloud2>("cluster_testing", 1, true);

  // Define PCL constants
  g_vg_leaf_sz = 0.0005; // VoxelGrid leaf size
  g_pt_thrs_min_x = -0.05; // PassThrough min threshold X
  g_pt_thrs_max_x = 0.05; // PassThrough max threshold X
  g_pt_thrs_min_y = -0.01; // PassThrough min threshold Y
  g_pt_thrs_max_y = 0.09; // PassThrough max threshold Y
  g_k_nn =
 50; // Normals nearest neighbor size

  ROS_INFO("cw1 class initialized");
}

// Task 1 callback - Pick and place a cube
bool cw1::t1Callback(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response)
{
  ROS_INFO("Task 1 callback triggered");


  geometry_msgs::Pose grasp_pose, basket_loc;
  grasp_pose = request.object_loc.pose;
  basket_loc.position = request.goal_loc.point;
  basket_loc.position.z += 0.4;
  grasp_pose.position.z += 0.22;

  double yaw = M_PI / 4;
  double pitch = M_PI;
  double roll = 0;

  ROS_INFO("yaw: %f", yaw);
  tf2::Quaternion quaternion;
  quaternion.setEulerZYX(yaw, pitch, roll);
  grasp_pose.orientation = tf2::toMsg(quaternion);
  basket_loc.orientation = grasp_pose.orientation;
  bool success1 = moveGripper(0.5);
  bool success5 = moveArm(grasp_pose);
  grasp_pose.position.z -= 0.1;
  bool success2 = moveArm(grasp_pose);
  bool success3 = moveGripper(0);
  // grasp_pose.position.z += 0.4;
  // bool success4 = moveArm(grasp_pose);
  bool success6 = moveArm(basket_loc);
  bool success7 = moveGripper(0.5);

  return true;
}

// Task 2 callback - Color detection
bool cw1::t2Callback(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)
{
  ROS_INFO("Task 2 callback triggered");

  // Get the number of target baskets
  int numb_targets = request.basket_locs.size();

  // Define the pose for the robot arm
  geometry_msgs::Pose target;
  target.orientation.x = 0.9239;
  target.orientation.y = -0.3827;
  target.orientation.z = 0;
  target.orientation.w = 0;

  // Initialize output vector
  std::vector<std::string> output(numb_targets);

  // Process each basket location
  for (int i = 0; i < numb_targets; i++) {
    target.position = request.basket_locs[i].point;
    target.position.z = 0.5;
    moveArm(target);

    // Initialize RGB values
    int r = 0;
    int g = 0;
    int b = 0;
    
    // Calculate average RGB values from the point cloud
    for (int j = 0; j < (*g_cloud_filtered2).points.size(); j++) {
      rgba = (*g_cloud_filtered2).points[j].rgba;
      uint8_t uint8_r = (rgba >> 16) & 0x0000ff;
      uint8_t uint8_g = (rgba >> 8) & 0x0000ff;
      uint8_t uint8_b = (rgba) & 0x0000ff;

      r += uint8_r;
      g += uint8_g;
      b += uint8_b;
    }
    
    // Average the values
    r = r / (*g_cloud_filtered2).points.size();
    g = g / (*g_cloud_filtered2).points.size();
    b = b / (*g_cloud_filtered2).points.size();

    // Determine the color based on RGB values
    if (r/255.0 <= red[0] + 0.1 && g/255.0 <= red[1] + 0.1 && b/255.0 <= red[2] + 0.1) {
      output[i] = "red";
    } else if (r/255.0 <= blue[0] + 0.1 && g/255.0 <= blue[1] + 0.1 && b/255.0 <= blue[2] + 0.1) {
      output[i] = "blue";
    } else if (r/255.0 <= purple[0] + 0.1 && g/255.0 <= purple[1] + 0.1 && b/255.0 <= purple[2] + 0.1) {
      output[i] = "purple";
    } else {
      output[i] = "none";
    }
  }

  // Set the response
  response.basket_colours = output;

  // Debug output
  for (const auto &s : output) {
    std::cout << s << std::endl;
  }

  return true;
}

// Task 3 callback - Sort cubes by color
bool cw1::t3Callback(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  ROS_INFO("Task 3 callback triggered");

  // Initialize flags
  cluster_or_not = false;
  task_3_triger = false;
  cluster_done = false;

  // Clear vectors
  red_cube.clear();
  blue_cube.clear();
  purple_cube.clear();
  red_basket.clear();
  blue_basket.clear();
  purple_basket.clear();


  // Define observation poses
  geometry_msgs::Pose target_pose_1, target_pose_2;
  target_pose_1.position.x = 0.41;
  target_pose_1.position.y = 0.3;
  target_pose_1.position.z = 0.7;
  target_pose_1.orientation.x = 0.9239;
  target_pose_1.orientation.y = -0.3827;
  target_pose_1.orientation.z = 0;
  target_pose_1.orientation.w = 0;

  target_pose_2 = target_pose_1;
  target_pose_2.position.y = -target_pose_1.position.y;

  // First observation position
  moveArm(target_pose_1);
  cluster_or_not = true;
  task_3_triger = true;

  // Wait for clustering to complete
  while (!cluster_done) {}

  // Second observation position
  cluster_or_not = false;
  task_3_triger = false;
  cluster_done = false;
  moveArm(target_pose_2);
  cluster_or_not = true;
  task_3_triger = true;

  // Wait for clustering to complete
  while (!cluster_done) {}
  cluster_or_not = false;
  task_3_triger = false;

  // Remove duplicate positions
  red_cube = discardRepeatPos(red_cube);
  blue_cube = discardRepeatPos(blue_cube);
  purple_cube = discardRepeatPos(purple_cube);
  red_basket = discardRepeatPos(red_basket);
  blue_basket = discardRepeatPos(blue_basket);
  purple_basket = discardRepeatPos(purple_basket);
  
  ROS_INFO("Duplicate positions removed");


  // Execute pick and place operations
  ROS_INFO("Starting pick and place operations");
  pickAndPlace(red_cube, red_basket, "red_cube");
  pickAndPlace(blue_cube, blue_basket, "blue_cube");
  pickAndPlace(purple_cube, purple_basket, "purple_cube");

  return true;
}

// Move arm to target pose
bool cw1::moveArm(geometry_msgs::Pose target_pose)
{
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  ROS_INFO("Planning path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) == 
                 moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");

  arm_group_.move();
  moveArm_done = true;

  return success;
}

// Control gripper width
bool cw1::moveGripper(float width)
{
  // Enforce safety limits
  if (width > gripper_open_)
    width = gripper_open_;
  if (width < gripper_closed_)
    width = gripper_closed_;

  // Calculate joint targets
  double eachJoint = width / 2.0;
  std::vector<double> gripperJointTargets(2, eachJoint);

  // Apply joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // Plan and execute
  ROS_INFO("Planning gripper movement");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
                 moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
  hand_group_.move();

  return success;
}


// Process point cloud for Task 2
void cw1::cloudCallBackOne(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Extract input point cloud info
  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;
    
  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2(g_pcl_pc, *g_cloud_ptr);

  // Apply filters to focus on specific area
  applyPT_x(g_cloud_ptr, g_cloud_filtered);
  applyPT_y(g_cloud_filtered, g_cloud_filtered2);

  // Publish filtered point cloud
  pubFilteredPCMsg(g_pub_task_2, *g_cloud_filtered2);
}

// Process point cloud for Task 3
void cw1::cloudCallBackTwo(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg_task_3)
{
  while (task_3_triger) {
    // Extract input point cloud info
    g_input_pc_frame_id_ = cloud_input_msg_task_3->header.frame_id;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_input_msg_task_3, g_pcl_pc_task_3);
    pcl::fromPCLPointCloud2(g_pcl_pc_task_3, *g_cloud_ptr_task_3);

    // Process the point cloud
    applyVX(g_cloud_ptr_task_3, g_cloud_filtered_task_3);
    findNormals(g_cloud_filtered_task_3);
    segPlane(g_cloud_filtered_task_3);
    segCylind(g_cloud_filtered_task_3);

    // Perform clustering if requested
    if (cluster_or_not) {
      cluster_or_not = false;
      cluster(g_cloud_cylinder);
    }

    // Publish filtered point cloud
    pcl::toROSMsg(*g_cloud_cylinder, g_cloud_filtered_msg);
    g_pub_task_3.publish(g_cloud_filtered_msg);
  }
}

// Apply voxel grid filter
void cw1::applyVX(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr)
{
  g_vx.setInputCloud(in_cloud_ptr);
  g_vx.setLeafSize(g_vg_leaf_sz, g_vg_leaf_sz, g_vg_leaf_sz);
  g_vx.filter(*out_cloud_ptr);
}

// Apply passthrough filter on X axis
void cw1::applyPT_x(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr)
{
  g_pt.setInputCloud(in_cloud_ptr);
  g_pt.setFilterFieldName("x");
  g_pt.setFilterLimits(g_pt_thrs_min_x, g_pt_thrs_max_x);
  g_pt.filter(*out_cloud_ptr);
}

// Apply passthrough filter on Y axis
void cw1::applyPT_y(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr)
{
  g_pt.setInputCloud(in_cloud_ptr);
  g_pt.setFilterFieldName("y");
  g_pt.setFilterLimits(g_pt_thrs_min_y, g_pt_thrs_max_y);
  g_pt.filter(*out_cloud_ptr);
}

// Publish filtered point cloud message
void cw1::pubFilteredPCMsg(ros::Publisher &pc_pub, PointC &pc)
{
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  pc_pub.publish(g_cloud_filtered_msg);
}

// Compute surface normals
void cw1::findNormals(PointCPtr &in_cloud_ptr)
{
  g_ne.setInputCloud(in_cloud_ptr);
  g_ne.setSearchMethod(g_tree_ptr);
  g_ne.setKSearch(g_k_nn);
  g_ne.compute(*g_cloud_normals);
}

// Segment planar surfaces
void cw1::segPlane(PointCPtr &in_cloud_ptr)
{
  // Configure plane segmentation
  g_seg.setOptimizeCoefficients(true);
  g_seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  g_seg.setNormalDistanceWeight(0.0001);
  g_seg.setMethodType(pcl::SAC_RANSAC);
  g_seg.setMaxIterations(1000);
  g_seg.setDistanceThreshold(0.03);
  g_seg.setInputCloud(in_cloud_ptr);
  g_seg.setInputNormals(g_cloud_normals);
  
  // Segment plane
  g_seg.segment(*g_inliers_plane, *g_coeff_plane);
  
  // Extract plane inliers
  g_extract_pc.setInputCloud(in_cloud_ptr);
  g_extract_pc.setIndices(g_inliers_plane);
  g_extract_pc.setNegative(false);
  g_extract_pc.filter(*g_cloud_plane);
  
  // Remove plane inliers, extract the rest
  g_extract_pc.setNegative(true);
  g_extract_pc.filter(*g_cloud_filtered2_task_3);
  g_extract_normals.setNegative(true);
  g_extract_normals.setInputCloud(g_cloud_normals);
  g_extract_normals.setIndices(g_inliers_plane);
  g_extract_normals.filter(*g_cloud_normals2);

  ROS_INFO_STREAM("PointCloud representing the planar component: " 
                 << g_cloud_plane->size() << " data points.");
}

// Segment cylindrical objects
void cw1::segCylind(PointCPtr &in_cloud_ptr)
{
  // Configure cylinder segmentation
  g_seg.setOptimizeCoefficients(true);
  g_seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  g_seg.setMethodType(pcl::SAC_RANSAC);
  g_seg.setNormalDistanceWeight(0.0001);
  g_seg.setMaxIterations(10000);
  g_seg.setDistanceThreshold(0.1);
  g_seg.setInputCloud(g_cloud_filtered2_task_3);
  g_seg.setInputNormals(g_cloud_normals2);
  
  // Segment cylinder
  g_seg.segment(*g_inliers_cylinder, *g_coeff_cylinder);
  
  // Extract cylinder inliers
  g_extract_pc.setInputCloud(g_cloud_filtered2_task_3);
  g_extract_pc.setIndices(g_inliers_cylinder);
  g_extract_pc.setNegative(false);
  g_extract_pc.filter(*g_cloud_cylinder);
  
  ROS_INFO_STREAM("PointCloud representing the cylinder component: " 
                 << g_cloud_cylinder->size() << " data points.");
  segCylind_done = true;
}

// Find cylinder pose
void cw1::findCylPose(PointCPtr &in_cloud_ptr)
{
  task_3_position.x = 0;
  task_3_position.y = 0;
  task_3_position.z = 0;

  // Compute centroid
  Eigen::Vector4f centroid_in;
  pcl::compute3DCentroid(*in_cloud_ptr, centroid_in);
  
  // Prepare pose message
  g_cyl_pt_msg.header.frame_id = g_input_pc_frame_id_;
  g_cyl_pt_msg.header.stamp = ros::Time(0);
  g_cyl_pt_msg.point.x = centroid_in[0];
  g_cyl_pt_msg.point.y = centroid_in[1];
  g_cyl_pt_msg.point.z = centroid_in[2];
  
  // Transform point to base frame
  geometry_msgs::PointStamped g_cyl_pt_msg_out;
  try {
    g_listener_.transformPoint("panda_link0", g_cyl_pt_msg, g_cyl_pt_msg_out);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("Received a transformation exception: %s", ex.what());
  }
  
  task_3_position = g_cyl_pt_msg_out.point;
}

// Cluster objects in point cloud
void cw1::cluster(PointCPtr &in_cloud_ptr)
{
  ROS_INFO("Start clustering");

  // Configure clustering
  g_tree_ptr->setInputCloud(in_cloud_ptr);
  g_ec.setClusterTolerance(0.02);
  g_ec.setMinClusterSize(100);
  g_ec.setMaxClusterSize(25000);
  g_ec.setSearchMethod(g_tree_ptr);
  g_ec.setInputCloud(in_cloud_ptr);
  
  // Perform clustering
  cluster_indices.clear();
  g_ec.extract(cluster_indices);

  // Process each cluster
  for (int i = 0; i < cluster_indices.size(); i++) {
    // Extract this cluster
    *singal_cluster_ptr = cluster_indices[i];
    g_extract_pc.setInputCloud(in_cloud_ptr);
    g_extract_pc.setIndices(singal_cluster_ptr);
    g_extract_pc.filter(*singal_cluster);

    // Calculate average color
    int r = 0, g = 0, b = 0;
    for (int j = 0; j < (*singal_cluster).points.size(); j++) {
      rgba = (*singal_cluster).points[j].rgba;
      r += (rgba >> 16) & 0x0000ff;
      g += (rgba >> 8) & 0x0000ff;
      b += (rgba) & 0x0000ff;
    }
    
    // Average the values
    r = r / (*singal_cluster).points.size();
    g = g / (*singal_cluster).points.size();
    b = b / (*singal_cluster).points.size();

    // Identify color and type (cube or basket)
    if (r/255.0 <= red[0]+0.05 && g/255.0 <= red[1]+0.05 && b/255.0 <= red[2]+0.05) {
      findCylPose(singal_cluster);
      if ((*singal_cluster).points.size() > 4500) {
        red_basket.push_back(task_3_position);
      } else {
        red_cube.push_back(task_3_position);
      }
    } else if (r/255.0 <= blue[0]+0.05 && g/255.0 <= blue[1]+0.05 && b/255.0 <= blue[2]+0.05) {
      findCylPose(singal_cluster);
      if ((*singal_cluster).points.size() > 4500) {
        blue_basket.push_back(task_3_position);
      } else {
        blue_cube.push_back(task_3_position);
      }
    } else if (r/255.0 <= purple[0]+0.05 && g/255.0 <= purple[1]+0.05 && b/255.0 <= purple[2]+0.05) {
      findCylPose(singal_cluster);
      if ((*singal_cluster).points.size() > 4500) {
        purple_basket.push_back(task_3_position);
      } else {
        purple_cube.push_back(task_3_position);
      }
    }

    moveArm_done = false;
  }

  ROS_INFO("Clustering finished");
  cluster_done = true;
}

// Publish cylinder pose
void cw1::publishPose(geometry_msgs::PointStamped &cyl_pt_msg)
{
  g_pub_pose.publish(cyl_pt_msg);
}

// Target pose callback
void cw1::targetPoseCallback(const geometry_msgs::PointStamped &cyl_pt_msg)
{
  cyl_pt_msg_header = cyl_pt_msg.header;
  cyl_pt_msg_point = cyl_pt_msg.point;
}

// Remove duplicate positions
std::vector<geometry_msgs::Point> cw1::discardRepeatPos(std::vector<geometry_msgs::Point> in_vector)
{
  std::vector<geometry_msgs::Point> temp_vec;
  geometry_msgs::Point temp_pos;
  bool same;

  if (in_vector.size() > 1) {
    // Check for duplicates
    for (int i = 0; i < in_vector.size()-1; i++) {
      same = false;
      temp_pos = in_vector[i];
      for (int j = i+1; j < in_vector.size(); j++) {
        if (abs(temp_pos.x-in_vector[j].x) < 0.01 && abs(temp_pos.y-in_vector[j].y) < 0.01) {
          same = true;
        }
      }
      if (!same) {
        temp_vec.push_back(temp_pos);
      }
    }
    temp_vec.push_back(in_vector[in_vector.size()-1]);

    // Sort by X coordinate
    int n = temp_vec.size();
    bool swap;
    do {
      swap = false;
      for (int k = 1; k < n; ++k) {
        if (temp_vec[k-1].x < temp_vec[k].x) {
          std::swap(temp_vec[k-1], temp_vec[k]);
          swap = true;
        }
      }
      --n;
    } while (swap);
  } else {
    temp_vec = in_vector;
  }

  return temp_vec;
}

// Print cube and basket information for debugging
void cw1::printCubeBasket()
{
  std::cout << "Object positions:" << std::endl;

  std::cout << "Red cubes:" << std::endl;
  for (int i = 0; i < red_cube.size(); i++) {
    std::cout << red_cube[i] << std::endl;
    std::cout << "--------------------" << std::endl;
  }

  std::cout << "Blue cubes:" << std::endl;
  for (int i = 0; i < blue_cube.size(); i++) {
    std::cout << blue_cube[i] << std::endl;
    std::cout << "--------------------" << std::endl;
  }

  std::cout << "Purple cubes:" << std::endl;
  for (int i = 0; i < purple_cube.size(); i++) {
    std::cout << purple_cube[i] << std::endl;
    std::cout << "--------------------" << std::endl;
  }

  std::cout << "Red baskets:" << std::endl;
  for (int i = 0; i < red_basket.size(); i++) {
    std::cout << red_basket[i] << std::endl;
    std::cout << "--------------------" << std::endl;
  }

  std::cout << "Blue baskets:" << std::endl;
  for (int i = 0; i < blue_basket.size(); i++) {
    std::cout << blue_basket[i] << std::endl;
    std::cout << "--------------------" << std::endl;
  }

  std::cout << "Purple baskets:" << std::endl;
  for (int i = 0; i < purple_basket.size(); i++) {
    std::cout << purple_basket[i] << std::endl;
    std::cout << "--------------------" << std::endl;
  }
}

// Pick and place operation
void cw1::pickAndPlace(std::vector<geometry_msgs::Point> cube, 
                     std::vector<geometry_msgs::Point> basket,
                     std::string cube_name)
{
  if (cube.size() > 0 && basket.size() > 0) {
    geometry_msgs::Pose tar_cube, tar_basket;

    // Set basket pose
    tar_basket.position = basket[0];
    tar_basket.position.z = 0.35;
    double yaw = M_PI / 4;
    double pitch = M_PI;
    double roll = 0;

    tf2::Quaternion quaternion;
    quaternion.setEulerZYX(yaw, pitch, roll);
    tar_basket.orientation = tf2::toMsg(quaternion);

    // Process each cube
    for (int i = 0; i < cube.size(); i++) {
      std::string name = cube_name + std::to_string(i+1);
      
      // Set cube pose
      tar_cube.position = cube[i];
      
      tar_cube.position.z += 0.22;

      double yaw = M_PI / 4;
      double pitch = M_PI;
      double roll = 0;

      tf2::Quaternion quaternion;
      quaternion.setEulerZYX(yaw, pitch, roll);
      tar_cube.orientation = tf2::toMsg(quaternion);
      
      
      tar_basket.orientation = tar_cube.orientation;
      moveGripper(0.5);
      moveArm(tar_cube);
      tar_cube.position.z -= 0.12;
      moveArm(tar_cube);
      moveGripper(0);
      tar_cube.position.z += 0.3;
      moveArm(tar_cube);
      moveArm(tar_basket);
      moveGripper(0.5);

      // Execute pick and place sequence
      // moveGripper(1);
      // tar_cube.position.z = 0.175;
      // moveArm(tar_cube);
      // tar_cube.position.z = 0.155;
      // moveArm(tar_cube);
      // moveGripper(0.01);
      // moveArm(tar_basket);
      // moveGripper(1);

      ROS_INFO("Pick and place operation completed");

      
    }
  }
}

