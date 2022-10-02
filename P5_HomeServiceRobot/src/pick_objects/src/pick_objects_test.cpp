#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void navigate_goal(MoveBaseClient &ac, double x, double y, double orientation, std::string action_name)
{
  ros::NodeHandle n;
  move_base_msgs::MoveBaseGoal goal;
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";//base_link
  goal.target_pose.header.stamp = ros::Time::now();
  n.setParam(action_name, false);
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = orientation;
  ROS_INFO_STREAM(action_name <<" Goal: Sending coordinates" );
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO_STREAM("Hooray, reached the " << action_name << " zone");
    n.setParam(action_name, true);
    }
  else 
  {
    ROS_INFO_STREAM("Failed to reach the " << action_name << " zone");
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
  navigate_goal(ac,2,5,1.0,"PickUp"); 
  // Wait 5 seconds to simulate picking up object
  ros::Duration(5).sleep();
  navigate_goal(ac,7.5,-5,0.5,"DropOff"); 
  ros::Duration(5).sleep();
  return 0;
}
