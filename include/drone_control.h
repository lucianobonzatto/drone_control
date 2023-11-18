/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#include "ros_client.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_listener.h>
#include <math.h>

class ROSClient; // Forward declaration because of circular reference

class DroneControl
{
  public:
    DroneControl(ROSClient *ros_client);
    ~DroneControl();

    static constexpr float TAKEOFF_ALTITUDE = 5.0;
    static constexpr float ROS_RATE = 20.0;
    static constexpr int   MAX_ATTEMPTS = 100;
    static constexpr int   KEEP_TIME = 100;
    static constexpr float TEST_FLIGHT_DURATION = 3.0; //In seconds per side
    static constexpr float TEST_FLIGHT_LENGTH = 2.0;   //In meters
    static constexpr int   TEST_FLIGHT_REPEAT = 2;     //Times
    static constexpr bool  KEEP_ALIVE = true;
    static constexpr double LAT_DEG_TO_M = 111000.0;
    static constexpr double LON_DEG_TO_M = 75000.0;

    // The setpoint publishing rate MUST be faster than 2Hz
    ros::Rate *rate_;
    tf2_ros::Buffer tfBuffer_;

    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped local_position_;
    sensor_msgs::NavSatFix global_position_;
    geometry_msgs::TransformStamped transformStamped_;

    void state_cb(const mavros_msgs::State::ConstPtr &msg);
    void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg);
    void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void global_position_cb(const sensor_msgs::NavSatFix::ConstPtr &msg);

    void flyToGlobal(double latitude, double longitude, double altitude, double yaw);
    void flyToLocal(double x, double y, double z);
    void hover(double seconds);
    
    void goMeter(double x, double y, double z);
    
    void offboardMode();
    void takeOff();
    void land();
    void disarm();

  private:
    uint8_t landed_state_ = 0;

    geometry_msgs::PoseStamped setpoint_pos_ENU_;
    geometry_msgs::PoseStamped endpoint_pos_ENU_;
    geometry_msgs::PoseStamped vision_pos_ENU_;
    geometry_msgs::PoseStamped gps_init_pos_;

    ros::Time last_request_;

    mavros_msgs::CommandBool arm_cmd_;
    std_msgs::String ewok_cmd_;

    ROSClient *ros_client_;

    double currentYaw();
    double distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2);
};


#endif /* DRONE_CONTROL_H */
