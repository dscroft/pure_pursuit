#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "pure_pursuit_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  rclcpp::spin(std::make_shared<pure_pursuit::PurePursuit>(options));
  rclcpp::shutdown();
  return 0;
}