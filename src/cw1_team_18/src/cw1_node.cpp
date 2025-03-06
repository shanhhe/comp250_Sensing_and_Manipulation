#include <cw1_class.h>

/**
 * @brief Main entry point for the cw1 solution
 */
int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "cw1_solution_node");
  ros::NodeHandle nh;

  // Create an instance of the cw1 class
  cw1 cw_class(nh);

  // Subscribe to point cloud data for task 2
  ros::Subscriber sub_cloud_task_2 = 
    nh.subscribe("/r200/camera/depth_registered/points",
                 1,
                 &cw1::cloudCallBackOne,
                 &cw_class);

  // Subscribe to point cloud data for task 3
  ros::Subscriber sub_cloud_task_3 = 
    nh.subscribe("/r200/camera/depth_registered/points",
                 1,
                 &cw1::cloudCallBackTwo,
                 &cw_class);
  
  // MoveIt! requirement for non-blocking group.move()
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Loop rate in Hz
  ros::Rate rate(10);

  // Main loop
  while (ros::ok()) {
    // Process all pending callbacks
    ros::spinOnce();

    // Sleep to fulfill the loop rate
    rate.sleep();
  }

  return 0;
}