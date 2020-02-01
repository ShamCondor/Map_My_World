#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ROS_INFO("Drive the robot - linear_x:%1.2f, angular_z:%1.2f", lin_x, ang_z);

    // Request the given velocities [lin_x, ang_z]
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the drive to target service and pass the requested velocities
    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive_to_target");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    bool bright_white_found = false;
    int pixel_index = -1;
    int white_pixel = 255;

    // Loop through each pixel in the image and check if there's a bright white one
    for (int i = 0; i < img.step * img.height; i+=3) {
        // check if all values are white (RGB)
        // RGB values of a pixel are arranged in a sequence in a row for all the pixels of that row
        if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel) {
            bright_white_found = true;
            pixel_index = i % img.step;
            break;
        }
    }

    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    if(bright_white_found)
    {
        
        int left_bound = img.step / 3;
        int right_bound = img.step - left_bound;
        // if ball was sighted on left side
        if(pixel_index < left_bound)
            drive_robot(0.1, 0.5);
        // ball was sighted directly in front of
        else if(pixel_index >= left_bound && pixel_index < right_bound)
            drive_robot(0.5, 0.0);
        // ball was sighted on right side
        else
            drive_robot(0.1, -0.5);
    }
    // Request a stop when there's no white ball seen by the camera
    else
        drive_robot(0.0, 0.0);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
