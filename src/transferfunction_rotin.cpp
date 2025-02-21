
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
  float linear_vel = 1, angular_vel = 0.5; //velocidade linear e angular do drone em m/s
  double command_interval = 4.0; //intervalo de tempo para cada parada do drone
  ros::Time last_command_time;
  std::vector<VelocityCommand> velocity_commands = {
      VelocityCommand(0.0, 0.0, 0.0, 0.0), //drone começa parado
      
      VelocityCommand(linear_vel, 0.0, 0.0, 0.0), //vai 1m/s em x
      VelocityCommand(linear_vel, 0.0, 0.0, 0.0), //vai 1m/s em x
      VelocityCommand(0.0, 0.0, 0.0, 0.0), //encerra comando. Drone fica parado
      VelocityCommand(0.0, 0.0, 0.0, 0.0), //encerra comando. Drone fica parado
      VelocityCommand(0.0, 0.0, 0.0, 0.0), //encerra comando. Drone fica parado
      VelocityCommand(0.0, 0.0, 0.0, 0.0), //encerra comando. Drone fica parado

      VelocityCommand(0.0, linear_vel, 0.0, 0.0), //vai 1m/s em y
      VelocityCommand(0.0, linear_vel, 0.0, 0.0), //vai 1m/s em y
      VelocityCommand(0.0, 0.0, 0.0, 0.0), //encerra comando. Drone fica parado
      VelocityCommand(0.0, 0.0, 0.0, 0.0), //encerra comando. Drone fica parado
      VelocityCommand(0.0, 0.0, 0.0, 0.0), //encerra comando. Drone fica parado
      VelocityCommand(0.0, 0.0, 0.0, 0.0), //encerra comando. Drone fica parado

      VelocityCommand(0.0, 0.0, linear_vel, 0.0), //vai 1m/s em z
      VelocityCommand(0.0, 0.0, linear_vel, 0.0), //vai 1m/s em z
      VelocityCommand(0.0, 0.0, 0.0, 0.0), //encerra comando. Drone fica parado
      VelocityCommand(0.0, 0.0, 0.0, 0.0), //encerra comando. Drone fica parado
      VelocityCommand(0.0, 0.0, 0.0, 0.0), //encerra comando. Drone fica parado
      VelocityCommand(0.0, 0.0, 0.0, 0.0), //encerra comando. Drone fica parado

      VelocityCommand(0.0, 0.0, 0.0, angular_vel), //rotaciona 0.5 m/s (yaw)
      VelocityCommand(0.0, 0.0, 0.0, angular_vel), //rotaciona 0.5 m/s (yaw)
      VelocityCommand(0.0, 0.0, 0.0, 0.0), //encerra comando. Drone fica parado
      VelocityCommand(0.0, 0.0, 0.0, 0.0), //encerra comando. Drone fica parado
      VelocityCommand(0.0, 0.0, 0.0, 0.0), //encerra comando. Drone fica parado
      VelocityCommand(0.0, 0.0, 0.0, 0.0), //encerra comando. Drone fica parado
      
      };

  drone_control.guidedMode();
  drone_control.takeOff();

  last_command_time = ros::Time::now();
  ROS_INFO("Start %d", ros::ok());
  ROS_INFO("index %d/%ld -> %f %f %f %f", index, velocity_commands.size(),
              velocity_commands[index].vel_x,
              velocity_commands[index].vel_y,
              velocity_commands[index].vel_z,
              velocity_commands[index].vel_r);
  while (ros::ok())
  {
    ros::Time current_time = ros::Time::now();
    if (index >= velocity_commands.size())
    {
      break;
    }
    const VelocityCommand command = velocity_commands[index];
    drone_control.cmd_vel(command.vel_x, command.vel_y, command.vel_z, command.vel_r);
    // drone_control.cmd_vel_unstamped(command.vel_x, command.vel_y, command.vel_z, command.vel_r);
    // drone_control.cmd_vel_base_link(command.vel_x, command.vel_y, command.vel_z, command.vel_r);

    if ((current_time - last_command_time).toSec() >= command_interval)
    {
      last_command_time = current_time;
      index++;

      if (index < velocity_commands.size())
      {
        ROS_INFO("index %d/%ld -> %f %f %f %f", index, velocity_commands.size(),
                    command.vel_x,
                    command.vel_y,
                    command.vel_z,
                    command.vel_r);
      }
    }

    ros::spinOnce();
    drone_control.rate_->sleep();
  }

  ROS_INFO("Stop");
  drone_control.land();
  return 0;
}
