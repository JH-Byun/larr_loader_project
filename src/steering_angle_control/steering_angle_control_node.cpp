#include "steering_angle_control/steering_angle_control.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SteeringAngleControl>());
    rclcpp::shutdown();
    return 0;
}