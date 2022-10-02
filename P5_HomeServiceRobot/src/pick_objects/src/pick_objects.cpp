#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher target_pub;

void navigateGoal(MoveBaseClient &ac, double x, double y, double orientation, std::string actionName)
{
  ros::NodeHandle n;
  move_base_msgs::MoveBaseGoal goal;
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";//base_link
  goal.target_pose.header.stamp = ros::Time::now();
  n.setParam(actionName, false);
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = orientation;
  ROS_INFO_STREAM(actionName <<" Goal: Sending coordinates" );
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO_STREAM("Hooray, reached the " << actionName << " zone");
    n.setParam(actionName, true);
    }
  else 
  {
    ROS_INFO_STREAM("Failed to reach the " << actionName << " zone");
   }
}

int main(int argc, char** argv){ 

  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  std_msgs::String actionMsg;
  ros::NodeHandle n;
  ros::Publisher actionPub = n.advertise<std_msgs::String>("actionName", 10);
  
  navigateGoal(ac,2,5,1.0,"PickUp"); 
  actionMsg.data = "PickUp";
  actionPub.publish(actionMsg);
  // Wait 5 seconds to simulate picking up object
  ros::Duration(5).sleep();
  navigateGoal(ac,7.5,-5,0.5,"DropOff");
  actionMsg.data = "DropOff";
  actionPub.publish(actionMsg);
  ros::Duration(20).sleep();
  return 0;
}
