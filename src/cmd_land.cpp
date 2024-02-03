#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <geometry_msgs/Pose2D.h>
#include <mavros_msgs/CommandTOL.h>
#include <time.h>
#include <cmath>
#include <math.h>
#include <ros/duration.h>

using namespace std;

// Set global variables
mavros_msgs::State current_state;
nav_msgs::Odometry current_pose;
std_msgs::Float64 current_heading;
std_msgs::String MODE;

// get armed state
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
  current_state = *msg;
}
// get current position of drone
void pose_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  current_pose = *msg;
  ROS_INFO("x: %f y: %f x: %f", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
}
//get compass heading
void heading_cb(const std_msgs::Float64::ConstPtr& msg)
{
  current_heading = *msg;
  //ROS_INFO("current heading: %f", current_heading.data);
}
void mode_cb(const std_msgs::String::ConstPtr& msg)
{
  MODE = *msg;
  ROS_INFO("current mode: %s", MODE.data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle controlnode;

  ros::Rate rate(5.0);
  ros::Subscriber state_sub = controlnode.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Publisher set_vel_pub = controlnode.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
  ros::Publisher local_pos_pub = controlnode.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::Subscriber currentPos = controlnode.subscribe<nav_msgs::Odometry>("mavros/global_position/local", 10, pose_cb);
  ros::Subscriber currentHeading = controlnode.subscribe<std_msgs::Float64>("mavros/global_position/compass_hdg", 10, heading_cb);
  ros::Subscriber mode_sub = controlnode.subscribe<std_msgs::String>("mode", 1, mode_cb);
  ros::ServiceClient arming_client = controlnode.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient takeoff_client = controlnode.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

  // wait for FCU connection
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Connected to FCU");
  // wait for mode to be set to guided
  while (current_state.mode != "OFFBOARD")
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Mode set to GUIDED");

  // arming
  mavros_msgs::CommandBool arm_request;
  arm_request.request.value = true;
  while (!current_state.armed && !arm_request.response.success)
  {
    ros::Duration(.1).sleep();
    arming_client.call(arm_request);
  }
  ROS_INFO("ARM sent %d", arm_request.response.success);

  // request takeoff
  mavros_msgs::CommandTOL takeoff_request;
  takeoff_request.request.altitude = 3;

  while (!takeoff_request.response.success)
  {
    ros::Duration(.1).sleep();
    takeoff_client.call(takeoff_request);
  }
  ROS_INFO("Takeoff initialized");
  sleep(10);
  ROS_INFO("hover");
  sleep(10);

  ROS_INFO("land");
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();

    if (true)
    {
      ros::ServiceClient land_client = controlnode.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
      mavros_msgs::CommandTOL srv_land;
      if (land_client.call(srv_land) && srv_land.response.success)
        ROS_INFO("land sent %d", srv_land.response.success);
      else
      {
        ROS_ERROR("Landing failed");
        //ros::shutdown();
        return -1;
      }

      while (ros::ok())
      {
        ros::spinOnce();
        rate.sleep();
      }
    }
    continue;

  }
  return 0;
}
