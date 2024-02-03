
#include "../include/drone_control.h"
#include "../include/ros_client.h"

#define HOVER_TIME 2
#define ALTITUDE 7

int main(int argc, char **argv)
{
  ROSClient ros_client(argc, argv);
  DroneControl drone_control(&ros_client);

  drone_control.guidedMode();
  drone_control.takeOff();

  drone_control.hover(HOVER_TIME);
  drone_control.flyToLocal(0, 0, ALTITUDE, 0);
  drone_control.hover(HOVER_TIME);
  drone_control.flyToLocal(4, 0, ALTITUDE, 0);
  drone_control.hover(HOVER_TIME);
  drone_control.flyToLocal(0, 0, ALTITUDE, 0);
  drone_control.hover(HOVER_TIME);

  drone_control.land();
  return 0;
}
