#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Defining a client node that will request the service published by the drive_bot node
ros::ServiceClient Client;

// This function calls the  service command_robot in order to drive the robot towards the given direction
void driving_robot(float linear_x, float angular_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = (float)linear_x;
    srv.request.angular_z = (float)angular_z;

    // Calling the service command_robot and further passing  the requested commands to motor to drive
    if (!Client.call(srv))
        ROS_ERROR("Failed to call the command_robot service ");
}

// This callback function executes in a loop and reads the image data each time the service is queried
void processimage_callback(const sensor_msgs::Image image)
{

    int pixel_in_white = 255;
    bool ball_found = false;
    int column_id = 0;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    for (int i = 0; i < image.height * image.step; i += 3)
    {
        red = image.data[i];
        green = image.data[i + 1];
        blue = image.data[i + 2];

        if ((red == pixel_in_white) && (green == pixel_in_white) && (blue == pixel_in_white))
        {
            column_id = i % image.step;
            // Note x towards left side is positive and right is negative
            if (column_id < image.step / 3)
                //the robot will drive towards left as column lies in the first step
                driving_robot(0.5, 1);
            else if (column_id < ((image.step * 2) / 3))
                //the robot will drive straight as column lies in the center step
                driving_robot(0.5, 0);
            else
                //the robot will drive right as column lies in the third step
                driving_robot(0.5, -1);
            ball_found = true;
            break;
        }
    }

    if (ball_found == false)
        driving_robot(0, 0);
}

int main(int argc, char **argv)
{
    // Initializing the client node : process_image and creating  a handle for ros master and other nodes to interact with it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service that is capable of requesting services using command_robot key word
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribing to the topic /camera/rgb/image_raw to extract the image raw pixels inside the processimage_callback
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, processimage_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}