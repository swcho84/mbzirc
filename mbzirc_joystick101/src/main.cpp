#include "mbzirc_joy_ctrl_ros2_lib.h"

using namespace std;
using namespace rclcpp;

// using SIGINT handler
void SigIntHandler(int param)
{
  printf("User pressed Ctrl+C..forced exit..\n");
  exit(1);
}

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/
int main(int argc, char** argv)
{
  // Set up ROS2
  init(argc, argv);

  // setting the node option, important, for using yaml file
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  // setting the node (nodeName, nodeOptions, Hz)
  auto node = std::make_shared<MbzircJoyCtrlRos2>("mbzirc_joystick101_node", options, 30);
  signal(SIGINT, SigIntHandler);

  spin(node);
  shutdown();

  return 0;
}