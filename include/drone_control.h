#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#include "ros_client.h"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

// // #include <math.h>

// // #include <geometry_msgs/PoseWithCovarianceStamped.h>
// // #include <geometry_msgs/PoseArray.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
// // #include "geometry_msgs/msg/pose_array.hpp"
// // #include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
// #include "geometry_msgs/msg/twist.hpp"

// #include "std_msgs/msg/string.hpp"

#include "mavros_msgs/srv/command_bool.hpp"
// #include "mavros_msgs/srv/command_tol.hpp"
// #include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/extended_state.hpp"
// #include "mavros_msgs/msg/global_position_target.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"

// #include "geographic_msgs/msg/geo_pose_stamped.hpp"

class ROSClient;

class DroneControl
{
public:
    DroneControl(ROSClient *ros_client);
    ~DroneControl();

    static constexpr float TAKEOFF_ALTITUDE = 3.0;
    static constexpr float ROS_RATE = 20.0;
    static constexpr int MAX_ATTEMPTS = 100;
    //     static constexpr int   KEEP_TIME = 100;
    //     static constexpr float TEST_FLIGHT_DURATION = 3.0; //In seconds per side
    //     static constexpr float TEST_FLIGHT_LENGTH = 2.0;   //In meters
    //     static constexpr int   TEST_FLIGHT_REPEAT = 2;     //Times
    //     static constexpr bool  KEEP_ALIVE = true;
    static constexpr double LAT_DEG_TO_M = 111000.0;
    static constexpr double LON_DEG_TO_M = 75000.0;

    // The setpoint publishing rate MUST be faster than 2Hz
    ROSClient *ros_client_;
    rclcpp::Rate *rate_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster br_;

    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped local_position_;
    sensor_msgs::msg::NavSatFix global_position_;
    geometry_msgs::msg::TransformStamped transformStamped_;
    uint8_t landed_state_;

    void state_cb(const mavros_msgs::msg::State::SharedPtr msg);
    void extended_state_cb(const mavros_msgs::msg::ExtendedState::SharedPtr msg);
    void local_position_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void global_position_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    void flyToGlobal(double latitude, double longitude, double altitude, double yaw);
    void flyToLocal(double x, double y, double z, double yaw);
    void hover(double seconds);

    void cmd_vel(double x, double y, double z, double ang);
    void cmd_vel_base_link(double x, double y, double z, double ang);
    void cmd_vel_unstamped(double x, double y, double z, double ang);

    void await_OFFBOARD_Mode();
    void set_OFFBOARD_Mode();
    
    void takeOff();
    void land();
    void disarm();
    void arm();

private:
    geometry_msgs::msg::PoseStamped setpoint_pos_ENU_;
    //     geometry_msgs::PoseStamped endpoint_pos_ENU_;
    //     geometry_msgs::PoseStamped vision_pos_ENU_;
    geometry_msgs::msg::PoseStamped gps_init_pos_;

    rclcpp::Time last_request_;

    mavros_msgs::srv::CommandBool arm_cmd_;
    //     std_msgs::String ewok_cmd_;

    double currentYaw();
    double distance(const geometry_msgs::msg::PoseStamped &p1, const geometry_msgs::msg::PoseStamped &p2);
};

#endif /* DRONE_CONTROL_H */
