#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <string>

using namespace std;

ros::Publisher marker_pub;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


void handle_marker(string name, int id, float x, float y, bool set_status)
{
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;


  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();


  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = name;
  marker.id = id;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  if(set_status) {
    marker.action = visualization_msgs::Marker::ADD;
  } else {
    marker.action = visualization_msgs::Marker::DELETE;
  }

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 0.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  marker_pub.publish(marker);

}


bool handle_pick_object(float x, float y, float w) {
  bool result = false;
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal robot_target_pose;


  // set up the frame parameters
  robot_target_pose.target_pose.header.frame_id = "map";
  robot_target_pose.target_pose.header.stamp = ros::Time::now();



  // Define a position and orientation for the robot to reach
  robot_target_pose.target_pose.pose.position.x = x;
  robot_target_pose.target_pose.pose.position.y = y;
  robot_target_pose.target_pose.pose.orientation.w = w;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal: (x: %f, y: %f, w: %f)", x, y, w);
  ac.sendGoal(robot_target_pose);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    //ROS_INFO("Hooray, the base moved 1 meter forward, goal 1 reached");
    result = true;
  } else {
    ROS_INFO("The base failed to move forward 1 meter for some reason, could not reach first goal");
    result = false;
  }

  return result;
}

int main( int argc, char** argv )
{
  float xStart = -1.0;
  float yStart = -2.0;
  float wStart = 1.0;

  float xGoal = -1.0;
  float yGoal = -4.0;
  float wGoal = 1.0;

  ros::init(argc, argv, "homeservice");


  ros::NodeHandle n;


  ros::Rate r(1);
  //ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  ROS_INFO("Create Marker at Start Point");
  //handle_marker("Marker-Start-Point" , 1, xStart, yStart, true);
  handle_marker("Marker-Start-Point" , 1, xStart, yStart, true);


  bool move_base_result = false;
  move_base_result = handle_pick_object(xStart, yStart, wStart);

  if(move_base_result) {
      ROS_INFO("Hooray, the base moved start point");
      handle_marker("Marker-Start-Point" , 1, xStart, yStart, false);
  } else {
      ROS_INFO("The base failed to move forward 1 meter for some reason, could not reach start point");
      return 0;
  }

  ROS_INFO("Waiting for 5 seconds");
  sleep(5.0);



  ROS_INFO("Move to Goal");
  move_base_result = false;

  move_base_result = handle_pick_object(xGoal, yGoal, wGoal);

  if(move_base_result) {
      ROS_INFO("Hooray, the base moved end point");
      handle_marker("Marker-Goal-Point" , 2, xGoal, yGoal, true);
  } else {
     return 0;
  }

  ROS_INFO("Waiting for 5 seconds");
  sleep(5.0);

  //handle_marker("Marker-Goal-Point" , 2, xGoal, yGoal, false);

  r.sleep();
  boost::mutex m;   
  m.unlock(); 
}
