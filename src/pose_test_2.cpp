
#include "../include/drone_control.h"
#include "../include/ros_client.h"

int main(int argc, char **argv)
{
  ROSClient ros_client(argc, argv);
  DroneControl drone_control(&ros_client);

  drone_control.guidedMode();
  drone_control.takeOff();
  int altitude = 6;

  drone_control.hover(5);
  drone_control.flyToLocal(0, 0, altitude, 0);
  drone_control.hover(5);
  drone_control.flyToLocal(2, 0, altitude, 0);
  drone_control.hover(5);
  drone_control.flyToLocal(2, 0, altitude, 0);
  drone_control.hover(5);
  drone_control.flyToLocal(-2, 0, altitude, 0);
  drone_control.hover(5);
  drone_control.flyToLocal(-2, 0, altitude, 0);
  drone_control.hover(5);

  drone_control.land();
  return 0;
}
