#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;



int main(int argc, char** argv){
  float xGoal1 = -1.0;
  float yGoal1 = -2.0;
  float wGoal1 = 1.0;


  float xGoal2 = -1.0;
  float yGoal2 = -3.0;
  float wGoal2 = 1.0;

  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();


  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = xGoal1;
  goal.target_pose.pose.position.y = yGoal1;
  goal.target_pose.pose.orientation.w = wGoal1;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending first goal: (x: %f, y: %f, w: %f)", xGoal1, yGoal1, wGoal1);
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the base moved 1 meter forward, goal 1 reached");
  } else {
    ROS_INFO("The base failed to move forward 1 meter for some reason, could not reach first goal");
    return 0;
  }

  // Wait 5 sec for to start second goal
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = xGoal2;
  goal.target_pose.pose.position.y = yGoal2;
  goal.target_pose.pose.orientation.w = wGoal2;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending first goal: (x: %f, y: %f, w: %f)", xGoal2, yGoal2, wGoal2);
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the base moved 1 meter forward, goal 2 reached");
  } else {
    ROS_INFO("The base failed to move forward 1 meter for some reason, could not reach second goal");
    return 0;
  }


  return 0;
}
