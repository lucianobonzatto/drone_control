
#include "../include/drone_control.h"
#include "../include/ros_client.h"

int main(int argc, char **argv)
{
  ROSClient ros_client(argc, argv);
  DroneControl drone_control(&ros_client);

  drone_control.guidedMode();

  drone_control.takeOff();
  int linear_velocity = 0.1;
  int angular_velocity = 0.1;

  drone_control.cmd_vel(0, 0, 0, 0);
  sleep(1);
  drone_control.cmd_vel(linear_velocity, 0, 0, angular_velocity);
  sleep(1);
  drone_control.cmd_vel(0, 0, 0, 0);

  drone_control.land();
  return 0;
}
