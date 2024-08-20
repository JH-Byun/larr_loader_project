#include "loader_sim/loader_sim.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoaderSim>());
  rclcpp::shutdown();
  return 0;
}