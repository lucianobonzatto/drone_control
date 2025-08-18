
#include "../include/drone_control.h"
#include "../include/ros_client.h"
#include "rclcpp/rclcpp.hpp"

struct VelocityCommand
{
  double vel_x;
  double vel_y;
  double vel_z;
  double vel_r;

  VelocityCommand(double x, double y, double z, double r)
      : vel_x(x), vel_y(y), vel_z(z), vel_r(r) {}
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ros_client_node");

  ROSClient ros_client(node);
  DroneControl drone_control(&ros_client);

  int index = 0;
  float linear_vel = 1, angular_vel = 1;
  double command_interval = 2.0;
  auto last_command_time = node->now();
  std::vector<VelocityCommand> velocity_commands = {
      VelocityCommand(0.0, 0.0, 0.0, 0.0),
      VelocityCommand(0.0, 0.0, linear_vel, 0.0),
      VelocityCommand(0.0, 0.0, 0.0, 0.0),
      VelocityCommand(0.0, 0.0, -linear_vel, 0.0),
      VelocityCommand(0.0, 0.0, 0.0, 0.0)
  };

  rclcpp::spin_some(node);
  drone_control.set_OFFBOARD_Mode();
  drone_control.takeOff();

  RCLCPP_INFO(node->get_logger(), "Start %d", rclcpp::ok());
  RCLCPP_INFO(node->get_logger(),
              "index %d/%ld -> %f %f %f %f", index, velocity_commands.size(),
              velocity_commands[index].vel_x,
              velocity_commands[index].vel_y,
              velocity_commands[index].vel_z,
              velocity_commands[index].vel_r);
  while (rclcpp::ok())
  {
    auto current_time = node->now();
    if (index >= velocity_commands.size())
    {
      break;
    }
    const VelocityCommand command = velocity_commands[index];
    // drone_control.cmd_vel(command.vel_x, command.vel_y, command.vel_z, command.vel_r);
    // drone_control.cmd_vel_unstamped(command.vel_x, command.vel_y, command.vel_z, command.vel_r);
    drone_control.cmd_vel_base_link(command.vel_x, command.vel_y, command.vel_z, command.vel_r);

    if ((current_time - last_command_time).seconds() >= command_interval)
    {
      last_command_time = current_time;
      index++;

      if (index < velocity_commands.size())
      {
        RCLCPP_INFO(node->get_logger(),
                    "index %d/%ld -> %f %f %f %f", index, velocity_commands.size(),
                    command.vel_x,
                    command.vel_y,
                    command.vel_z,
                    command.vel_r);
      }
    }

    rclcpp::spin_some(node);
    drone_control.rate_->sleep();
  }

  RCLCPP_INFO(node->get_logger(), "Stop");
  drone_control.land();
  return 0;
}
