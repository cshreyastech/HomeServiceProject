#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <string>

using namespace std;

ros::Publisher marker_pub;


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



int main( int argc, char** argv )
{
  float xStart = -1.0;
  float yStart = -2.0;

  float xGoal = -1.0;
  float yGoal = -3.0;

  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;

  ros::Rate r(1);
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

  // Wait 5 sec
  ROS_INFO("Waiting for 5 seconds to delete marker");
  sleep(5.0);


  handle_marker("Marker-Start-Point" , 1, xStart, yStart, false);

  ROS_INFO("Waiting for 5 seconds");
  sleep(5.0);



  ROS_INFO("Create Marker at Goal Point");
  //handle_marker("Marker-Start-Point" , 1, xStart, yStart, true);
  handle_marker("Marker-Start-location" , 2, xGoal, yGoal, true);

  // Wait 5 sec
  ROS_INFO("Waiting for 5 seconds to delete marker");
  sleep(5.0);

  handle_marker("Marker-Goal-location" , 2, xGoal, yGoal, false);
  sleep(5.0);

  r.sleep();
}
