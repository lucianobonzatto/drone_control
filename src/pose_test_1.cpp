
#include "../include/drone_control.h"
#include "../include/ros_client.h"

#define HOVER_TIME 2
#define ALTITUDE 7

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ros_client_node");

  ROSClient ros_client(node);
  DroneControl drone_control(&ros_client);
  rclcpp::spin_some(node);

  drone_control.set_OFFBOARD_Mode();
  drone_control.takeOff();

  drone_control.hover(HOVER_TIME);
  drone_control.flyToLocal(0, 0, ALTITUDE, 0);
  drone_control.hover(HOVER_TIME);
  drone_control.flyToLocal(2, 0, ALTITUDE, 0);
  drone_control.hover(HOVER_TIME);
  drone_control.flyToLocal(4, 0, ALTITUDE, 0);
  drone_control.hover(HOVER_TIME);
  drone_control.flyToLocal(0, 0, ALTITUDE, 0);
  drone_control.hover(HOVER_TIME);

  drone_control.land();
  return 0;
}
