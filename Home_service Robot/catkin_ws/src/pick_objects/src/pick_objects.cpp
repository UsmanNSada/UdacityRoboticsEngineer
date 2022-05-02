#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the first frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 4.31;
  goal.target_pose.pose.position.y = 4.18;
  goal.target_pose.pose.orientation.w = 1.00;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("moving to pickup first goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its first goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("The robot has picked up the virtual object");

    // wait for 5 seconds at the drop-off zone
    ac.waitForResult(ros::Duration(5.0));

    // Define drop-off position and orientation for the robot to reach
    goal.target_pose.pose.position.x = -5.63;
    goal.target_pose.pose.position.y = -4.04;
    goal.target_pose.pose.orientation.w = 1.00;

     // Send the drop-off position and orientation for the robot to reach
    ROS_INFO("Robot is travelling to drop-off zone");
    ac.sendGoal(goal);

     // Wait an infinite time for the results
    ac.waitForResult();

    // check if the robot has reached the dropoff zone
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Robot has successfully travelled to drop-off zone");   
    else
      ROS_INFO("The base failed to move to the target location");
  }
  else{
    ROS_INFO("The base failed to move forward 1 meter for some reason");
  }
       
  return 0;
}