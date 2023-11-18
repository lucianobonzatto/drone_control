#include "drone_control.h"

#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TwistStamped.h>

DroneControl::DroneControl(ROSClient *ros_client)
{
  this->ros_client_ = ros_client;
  this->ros_client_->init(this);

  // The setpoint publishing rate MUST be faster than 2Hz
  this->rate_ = new ros::Rate(ROS_RATE);

  static tf2_ros::TransformListener tfListener(tfBuffer_);
}

void DroneControl::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
  current_state_ = *msg;
}

void DroneControl::extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg)
{
  landed_state_ = msg->landed_state;
}

void DroneControl::local_position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  local_position_ = *msg;
  static int cnt = 0;

  static tf2_ros::TransformBroadcaster br;
  static tf2_ros::StaticTransformBroadcaster sbr;

  // Transformation from world to drone
  transformStamped_.header.stamp = local_position_.header.stamp;
  transformStamped_.header.frame_id = "world";
  transformStamped_.child_frame_id = "drone";
  transformStamped_.transform.translation.x = local_position_.pose.position.x;
  transformStamped_.transform.translation.y = local_position_.pose.position.y;
  transformStamped_.transform.translation.z = local_position_.pose.position.z;
  transformStamped_.transform.rotation = local_position_.pose.orientation;
  br.sendTransform(transformStamped_);

  cnt++;
  if(cnt % 100 == 0)
  {
    ROS_INFO("Mavros local position: E: %f, N: %f, U: %f, yaw: %f", transformStamped_.transform.translation.x,
             transformStamped_.transform.translation.y, transformStamped_.transform.translation.z, currentYaw());
  }
}

DroneControl::~DroneControl()
{
  geometry_msgs::TwistStamped vel_msg;
  
  vel_msg.header.stamp = ros::Time::now();
  vel_msg.twist.linear.x = 0;
  vel_msg.twist.linear.y = 0;
  vel_msg.twist.linear.z = 0;
  vel_msg.twist.angular.x = 0;
  vel_msg.twist.angular.y = 0;
  vel_msg.twist.angular.z = 0;
  ros_client_->velocity_pub.publish(vel_msg);

  // this->land();

}

void DroneControl::global_position_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
  global_position_ = *msg;
  static int cnt = 0;

  cnt++;
  if(cnt % 100 == 0)
  {
    //ROS_INFO("GPS: lat: %f, long: %f, alt: %f", msg->latitude, msg->longitude, msg->altitude);
  }
}

void DroneControl::flyToGlobal(double latitude, double longitude, double altitude, double yaw)
{
  mavros_msgs::GlobalPositionTarget target;
  target.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
  target.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_VX |
      mavros_msgs::GlobalPositionTarget::IGNORE_VY |
      mavros_msgs::GlobalPositionTarget::IGNORE_VZ |
      mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
      mavros_msgs::GlobalPositionTarget::IGNORE_AFY |
      mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
      mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;
  target.latitude = latitude;
  target.longitude = longitude;
  target.altitude = altitude;
  target.yaw = yaw;

  while(ros::ok() &&
         ( fabs(latitude - global_position_.latitude)*LAT_DEG_TO_M > 1.0
        || fabs(longitude - global_position_.longitude)*LON_DEG_TO_M > 1.0
        || fabs(altitude - global_position_.altitude) > 1.0))
  {
    ROS_INFO("Dist: lat: %f, long: %f, alt: %f", fabs(latitude - global_position_.latitude)*LAT_DEG_TO_M, fabs(longitude - global_position_.longitude)*LON_DEG_TO_M, altitude-global_position_.altitude);
    ros_client_->global_setpoint_pos_pub_.publish(target);
    ros::spinOnce();
    rate_->sleep();
  }
}

void DroneControl::flyToLocal(double x, double y, double z)
{
  double yaw = 0;
  if(!std::isfinite(yaw))
  {
    yaw = currentYaw();
    ROS_INFO("Flying to local coordinates E: %f, N: %f, U: %f, current yaw: %f", x, y, z, yaw);
  }
  else ROS_INFO("Flying to local coordinates E: %f, N: %f, U: %f, yaw: %f", x, y, z, yaw);

  setpoint_pos_ENU_.pose.position.x = x;
  setpoint_pos_ENU_.pose.position.y = y;
  setpoint_pos_ENU_.pose.position.z = z;
  setpoint_pos_ENU_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

  while(ros::ok() && distance(setpoint_pos_ENU_, local_position_) > 0.5)
  {
    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
    rate_->sleep();
  }

  //Publish for another second
  for(int i = 0; ros::ok() && i < ROS_RATE/20; ++i)
  {
    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
    rate_->sleep();
  }
}

void DroneControl::hover(double seconds)
{
  ROS_INFO("Hovering for %f seconds in position: E: %f, N: %f, U: %f", seconds,
           local_position_.pose.position.x,
           local_position_.pose.position.y,
           local_position_.pose.position.z);

  for(int i = 0; ros::ok() && i < 15 * ROS_RATE; ++i)
  {
    ros_client_->setpoint_pos_pub_.publish(local_position_);
    ros::spinOnce();
    rate_->sleep();
  }

  return;
}

void DroneControl::goMeter(double x, double y, double z)
{
  geometry_msgs::PoseStamped initial_position = local_position_;
  geometry_msgs::TwistStamped vel_msg;
  
  vel_msg.header.stamp = ros::Time::now();
  vel_msg.twist.linear.x = x;
  vel_msg.twist.linear.y = y;
  vel_msg.twist.linear.z = z;
  vel_msg.twist.angular.x = 0;
  vel_msg.twist.angular.y = 0;
  vel_msg.twist.angular.z = 0;

  ROS_INFO("Going one meter to: x: %f, y: %f, z: %f",
           vel_msg.twist.linear.x,
           vel_msg.twist.linear.y,
           vel_msg.twist.linear.z);

  for(int i = 0; i<MAX_ATTEMPTS && ros::ok() && distance(initial_position, local_position_) < 10; ++i)
  {
    ros_client_->velocity_pub.publish(vel_msg);
    ros::spinOnce();
    rate_->sleep();
  }
  vel_msg.twist.linear.x = 0;
  vel_msg.twist.linear.y = 0;
  vel_msg.twist.linear.z = 0;

  for(int i = 0; i<MAX_ATTEMPTS && ros::ok(); ++i)
  {
    ros_client_->velocity_pub.publish(vel_msg);
    ros::spinOnce();
    rate_->sleep();
  }
  ROS_INFO("Movement finished!");

  return;
}

void DroneControl::offboardMode()
{
  // Wait for FCU connection
  while(ros::ok() && current_state_.connected)
  {
    ros::spinOnce();
    rate_->sleep();
    ROS_INFO("connecting to FCU...");
  }

  // Wait for ROS
  for(int i = 0; ros::ok() && i < 4 * ROS_RATE; ++i)
  {
    ros::spinOnce();
    rate_->sleep();
  }

  ROS_INFO("Switching to GUIDED mode");

  if(ros::Time::now() - local_position_.header.stamp < ros::Duration(1.0))
  {
    ROS_INFO("Local_position available");
  }
  else
  {
    ROS_WARN("Local_position not available, initializing to 0");
    local_position_.header.stamp = ros::Time::now();
    local_position_.header.frame_id = "world";
    local_position_.pose.position.x = 0;
    local_position_.pose.position.y = 0;
    local_position_.pose.position.z = 0;
    local_position_.pose.orientation.x = 0;
    local_position_.pose.orientation.y = 0;
    local_position_.pose.orientation.z = 0;
    local_position_.pose.orientation.w = 1;
  }

  setpoint_pos_ENU_ = gps_init_pos_ = local_position_;

  // Send a few setpoints before starting, otherwise px4 will not switch to GUIDED mode
  for(int i = 20; ros::ok() && i > 0; --i)
  {
    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
    rate_->sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "GUIDED";

  arm_cmd_.request.value = true;

  last_request_ = ros::Time::now();

  // Change to GUIDED mode and arm
  while(ros::ok() && !current_state_.armed)
  {
    if( current_state_.mode != "GUIDED" && (ros::Time::now() - last_request_ > ros::Duration(5.0)))
    {
      ROS_INFO("%s",current_state_.mode.c_str());
      if( ros_client_->set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent)
      {
        ROS_INFO("GUIDED enabled");
      }
      last_request_ = ros::Time::now();
    }
    else
    {
      if( !current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0)))
      {
        if( ros_client_->arming_client_.call(arm_cmd_) && arm_cmd_.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
      }
    }
    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
    rate_->sleep();
  }

  return;
}

void DroneControl::takeOff()
{
  mavros_msgs::CommandTOL takeoff_cmd;
  takeoff_cmd.request.yaw = 0;
  takeoff_cmd.request.latitude = NAN; //Land at current location
  takeoff_cmd.request.longitude = NAN;
  takeoff_cmd.request.altitude = TAKEOFF_ALTITUDE;

  ROS_INFO("Trying to Takeoff");
  while(!(ros_client_->takeoff_client_.call(takeoff_cmd) && takeoff_cmd.response.success))
  {
    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
    ROS_WARN("Retrying to Takeoff");
    rate_->sleep();
  }
  ROS_INFO("Takeoff finished!");
  return;
}

void DroneControl::land()
{
  int i;
  mavros_msgs::CommandTOL land_cmd;
  land_cmd.request.yaw = 0;
  land_cmd.request.latitude = NAN; //Land at current location
  land_cmd.request.longitude = NAN;
  land_cmd.request.altitude = 0;

  ROS_INFO("Trying to land");
  while(!(ros_client_->land_client_.call(land_cmd) && land_cmd.response.success))
  {
    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
    ROS_WARN("Retrying to land");
    rate_->sleep();
  }

  // Wait until proper landing (or a maximum of 15 seconds)
  for(i = 0; ros::ok() && landed_state_ != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND && i < MAX_ATTEMPTS; ++i)
  {
    ros::spinOnce();
    rate_->sleep();
  }
  if(i == MAX_ATTEMPTS)
    ROS_WARN("Landing failed, aborting");
  else
    ROS_INFO("Landing success");


  return;
}

void DroneControl::disarm()
{
  // Disarm
  arm_cmd_.request.value = false;
  while(ros::ok() && current_state_.armed)
  {
    if( current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0)))
    {
      if( ros_client_->arming_client_.call(arm_cmd_) && arm_cmd_.response.success)
      {
        ROS_INFO("Vehicle disarmed");
      }
      last_request_ = ros::Time::now();
    }
    ros::spinOnce();
    rate_->sleep();
  }
  return;
}

double DroneControl::currentYaw()
{
  //Calculate yaw current orientation
  double roll, pitch, yaw;
  tf::Quaternion q;

  tf::quaternionMsgToTF(local_position_.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  return yaw;
}

double DroneControl::distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2)
{
  tf::Point t1, t2;
  tf::pointMsgToTF(p1.pose.position, t1);
  tf::pointMsgToTF(p2.pose.position, t2);

  return t1.distance(t2);
}
