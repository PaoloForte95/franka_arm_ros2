#include <memory>

#include "franka_moveit_controller/moveit_controller.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<franka_example_controllers::MoveItContoller>("moveit_controller");
  rclcpp_lifecycle::State state;
  node->on_activate(state);
  node->on_configure(state);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}