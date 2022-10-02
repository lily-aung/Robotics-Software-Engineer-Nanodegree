#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include <math.h>
#include "std_msgs/String.h"

double pickUpGoal[3]  = {6.7  , 6.9  , 0 } ; //{2  , 5 , 0} : MAP;
double dropOffGoal[3] = {13.04, -2.67, 0 } ;//{7.5,-5 , 0} : MAP;
bool pickUp = false;
bool dropOff = false;
double poseX = 0.0 , poseY = 0.0;
std::string actionName ;

double computeDistance(double tolerance)
{
  double dx1 = fabs(pickUpGoal[0] - poseX);
  double dy1 = fabs(pickUpGoal[1] - poseY);
  double pickUpDist = sqrt(dx1*dx1 + dy1*dy1);
  double dx2 = fabs(dropOffGoal[0] - poseX);
  double dy2 = fabs(dropOffGoal[1] - poseY);  
  double dropOffDist = sqrt(dx2*dx2 + dy2*dy2);
  if (pickUpDist<=tolerance) { 
    pickUp = true; 
  }
  if (dropOffDist<=tolerance) { 
    dropOff = true; 
  }
  //ROS_INFO_STREAM( "ODOM Pose x : " << poseX <<" ,  y: " << poseY);
  //ROS_INFO_STREAM( "PickupDist : " << pickUpDist <<" ,  DropOffDist: " << dropOffDist);
  //ROS_INFO_STREAM(" PickUp " << pickUp << " : DropOff " << dropOff);
  }

void odomGetPose(const nav_msgs::Odometry::ConstPtr& msg)
{
  ::poseX = msg->pose.pose.position.x;
  ::poseY = msg->pose.pose.position.y;
  computeDistance(0.4);
}

void getActionName(const std_msgs::String::ConstPtr& msg)
{
  actionName = msg->data.c_str();
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Subscriber odom_pose_sub = n.subscribe("/odom", 1000, odomGetPose);
  ros::Subscriber action_sub = n.subscribe("/actionName", 10, getActionName);                  
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  uint32_t sphere_shape = visualization_msgs::Marker::SPHERE;
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time::now();
  marker.ns = "add_markers";
  marker.id = 0;
  marker.type = sphere_shape;
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = pickUpGoal[0];
  marker.pose.position.y = pickUpGoal[1];
  marker.pose.position.z = pickUpGoal[2];
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  actionName = "Pick Up";
  while (ros::ok())
  {
    while (marker_pub.getNumSubscribers() < 1)
    {
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(10);
      ROS_INFO_STREAM("Robot is travelling to the " << actionName << " zone");
    }
    
   
    if (dropOff)
    {
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = dropOffGoal[0];
      marker.pose.position.y = dropOffGoal[1];
      marker.pose.position.z = dropOffGoal[2];       
      ROS_INFO("Robot dropped virtual object");
      actionName = "Drop Off";
    }
    if (pickUp)
    {
      marker.action = visualization_msgs::Marker::DELETE;
      ROS_INFO("Robot picked up virtual object");
      sleep(3);
      pickUp = false;
      ROS_INFO("Robot is travelling to the Drop off zone");
    }
    
    marker_pub.publish(marker);
    ros::spinOnce();
    sleep(5);

  }
  return 0;
}