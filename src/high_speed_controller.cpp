#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <iostream>
#include <tf/transform_listener.h>

/*
 CAK
high_speed_controller.cpp
BLG456E Assignment 2 skeleton

Instructions: Edit the control loop below.
Read further instructions

*/

//contains waypoint data
geometry_msgs::Transform waypoint;

//the data structure that will receive the current pose
//the "stamped" means simply that there is time-stamp information available in the data structure's fields
tf::StampedTransform robot_pose;

//for containing the motor commands to send to the robot
geometry_msgs::Twist motor_command;

//waypoint callback
void waypoint_callback(const geometry_msgs::Transform::ConstPtr &msg) // <--- callback
{

    //***************************************
    //***          Obtain current destination
    //***************************************

    //save waypoint data for printing out in main loop
    waypoint = *msg;
}

int main(int argc, char **argv) {

    //setup ROS node, subscribe waypoint_cb to the topic /waypoint_cmd & publish motor commands
    ros::init(argc, argv, "schumacher_456");
    ros::NodeHandle n;
    ros::Subscriber waypoint_subscriber = n.subscribe("/waypoint_cmd", 1000, waypoint_callback); // <--- set up callback
    ros::Publisher motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    //you could in principle also subscribe to the laser scan as is done in assignment 1.

    //setup transform cache manager
    tf::TransformListener listener;

    //start a loop; one loop per two second
    ros::Rate delay(0.5); // perhaps this could be faster for a controller?
    while (ros::ok()) {

        //***************************************
        //***          Obtain current robot pose
        //***************************************

        ros::spinOnce(); // may be needed to call the callback

        try {
            //grab the latest available transform from the odometry frame (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
            listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), robot_pose);
        }
        //if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        //***************************************
        //***          Print current robot pose
        //***************************************

        //Print out the x,y coordinates of the transform
        std::cout << "Robot is believed to be at (x,y): (" << robot_pose.getOrigin().x() << "," << robot_pose.getOrigin().y() << ")" << std::endl;

        //Convert the quaternion-based orientation of the latest message to angle-axis in order to get the z rotation & print it.
        tf::Vector3 robot_axis = robot_pose.getRotation().getAxis();
        double robot_theta = robot_pose.getRotation().getAngle() * robot_axis[2]; // only need the z axis
        std::cout << "Robot is believed to have orientation (theta): (" << robot_theta << ")" << std::endl
                  << std::endl;

        //***************************************
        //***          Print current destination
        //***************************************

        // the curr_waypoint variable is filled in in the waypoint_callback function above, which comes from incoming messages
        // subscribed to in the .subscribe function call above.

        //Print out the x,y coordinates of the latest message
        std::cout << "Current waypoint (x,y): (" << waypoint.translation.x << "," << waypoint.translation.y << ")" << std::endl;

        //Convert the quaternion-based orientation of the latest message to angle-axis in order to get the z rotation & print it.
        tf::Quaternion quat(waypoint.rotation.x, waypoint.rotation.y, waypoint.rotation.z, waypoint.rotation.w);
        tf::Vector3 waypoint_axis = quat.getAxis();
        double waypoint_theta = quat.getAngle() * waypoint_axis[2]; // only need the z axis
        std::cout << "Current waypoint (theta): (" << waypoint_theta << ")" << std::endl
                  << std::endl;

        //***************************************
        //***          DRIVE THE ROBOT HERE (same as with assignment 1)
        //***************************************

        //I don't know what to do because nobody has programmed me with any smartness,
        //so I'll do what everybody does, and drive very fast straight forwards.
        motor_command.linear.x = 5000.0;
        motor_command.angular.z = 0.0;

        motor_command_publisher.publish(motor_command);

        //FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME

        delay.sleep();
    }
    return 0;
}
