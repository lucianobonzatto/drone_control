
#include "../include/drone_control.h"
#include "../include/ros_client.h"

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
  ROSClient ros_client(argc, argv);
  DroneControl drone_control(&ros_client);

  int index = 0;
  float linear_vel = 1, angular_vel = 1;
  double command_interval = 1.0;
  ros::Time last_command_time;
  std::vector<VelocityCommand> velocity_commands = {
      VelocityCommand(0.0, 0.0, 0.0, 0.0),
      VelocityCommand(linear_vel, 0.0, 0.0, 0.0),
      VelocityCommand(0.0, 0.0, 0.0, 0.0)};

  drone_control.guidedMode();
  drone_control.takeOff();
  drone_control.hover(1);

  last_command_time = ros::Time::now();
  while (ros::ok())
  {
    ros::Time current_time = ros::Time::now();
    if (index < velocity_commands.size())
    {
      break;
    }

    if ((current_time - last_command_time).toSec() >= command_interval)
    {
      const VelocityCommand command = velocity_commands[index];
      drone_control.cmd_vel_base_link(command.vel_x, command.vel_y, command.vel_z, command.vel_r);
      last_command_time = current_time;
      index++;
    }

    drone_control.hover(1);
    ros::spinOnce();
  }

  drone_control.land();
  return 0;
}