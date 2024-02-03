#ifndef ROS_CLIENT_H
#define ROS_CLIENT_H

#include "drone_control.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class DroneControl; // Forward declaration because of circular reference

class ROSClient
{
  public:
    ROSClient(int &argc, char **argv);

    void init(DroneControl *const drone_control);

    ros::Subscriber state_sub_;
    ros::Subscriber extended_state_sub_;
    ros::Subscriber local_pos_sub_;
    ros::Subscriber global_pos_sub_;
    ros::Subscriber setpoint_pos_sub_;

    ros::Publisher global_setpoint_pos_pub_;
    ros::Publisher setpoint_pos_pub_;
//    ros::Publisher vision_pos_pub_;
    ros::Publisher velocity_pub;
    ros::Publisher velocity_unstamped_pub;

    ros::ServiceClient arming_client_;
    ros::ServiceClient land_client_;
    ros::ServiceClient takeoff_client_;
    ros::ServiceClient set_mode_client_;

    void setParam(const std::string &key, double d);

    bool avoidCollision_;

  private:
    ros::NodeHandle *nh_;
};

#endif /* ROS_CLIENT_H */
