
#include "include/drone_control.h"
#include "include/ros_client.h"

int main(int argc, char **argv)
{
  ROSClient ros_client(argc, argv);
  DroneControl drone_control(&ros_client);

  drone_control.offboardMode();

  drone_control.takeOff();

  drone_control.flyToLocal(0, 0, 8);
  drone_control.flyToLocal(4, 0, 8);
  drone_control.flyToLocal(4, 4, 8);
  drone_control.flyToLocal(0, 4, 8);
  drone_control.flyToLocal(0, 0, 8);
  drone_control.flyToLocal(0, 0, 10);
  drone_control.flyToLocal(0, 0, 4);

  //drone_control.hover(10);

  drone_control.land();
  //drone_control.disarm();

  while(ros::ok() && DroneControl::KEEP_ALIVE)
  {
    ros::spin();
  }

  return 0;
}
