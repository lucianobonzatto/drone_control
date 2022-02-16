
#include "include/drone_control.h"
#include "include/ros_client.h"

int main(int argc, char **argv)
{
  ROSClient ros_client(argc, argv);
  DroneControl drone_control(&ros_client);

  drone_control.offboardMode();

  drone_control.takeOff();

  drone_control.goMeter(0, 0, 0);
//  drone_control.goMeter(0, 0, 0);
//  drone_control.goMeter(1, 0, 0);
//  drone_control.goMeter(0, 0, 1);

  //drone_control.flyToLocal(4.0, -5.0, DroneControl::SAFETY_ALTITUDE_GPS);

//  drone_control.hover(10);

  drone_control.land();
  //drone_control.disarm();

  while(ros::ok() && DroneControl::KEEP_ALIVE)
  {
    ros::spin();
  }

  return 0;
}
