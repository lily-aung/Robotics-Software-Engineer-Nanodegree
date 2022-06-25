#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request a service
ros::ServiceClient client;
// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Sending request"); 
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = (float)lin_x;
    srv.request.angular_z = (float)ang_z;
	
    // Call the service
    if (!client.call(srv))
        ROS_ERROR("Failed to call service: /ball_chaser/command_robot");
}

// The callback function continuously executes and read the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    enum BallPosition { NONE, LEFT, MID, RIGHT} ball_position = NONE;
    int idx = 0;
    for (int i=0; i < img.height * img.step; i += 3)
    {
        if ((img.data[i] == 255) && (img.data[i+1] == 255) && (img.data[i+2] == 255))
        {
            idx = i % img.step;
            if (idx < img.step/3)
                ball_position = LEFT;
            else if (idx < (img.step/3 * 2))
                ball_position = MID;
            else
                ball_position = RIGHT;
            break;
         }
	else
		ball_position = NONE;
     }
    switch (ball_position) 
    {
        case LEFT:
            drive_robot(0.5, 1);
            break;
        case MID:
            drive_robot(0.5, 0); 
             break;
        case RIGHT:
            drive_robot(0.5, -1);
            break;
        case NONE:
            drive_robot(0, 0); 
            break;
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subsribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
