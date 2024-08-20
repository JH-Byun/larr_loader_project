#include "loader_sim/loader_sim.hpp"
#include "so3_utils.hpp"

void LoaderSim::topic_publisher()
{
    builtin_interfaces::msg::Time sensor_stamp_ = this->now();

    // publish the steering angle
    auto steering_angle_msg = sensor_msgs::msg::JointState();
    // sensor_msgs::msg::JointState steering_angle_msg;
    steering_angle_msg.header.frame_id = "steering angle";
    steering_angle_msg.header.stamp = sensor_stamp_;
    steering_angle_msg.name = {"steering_angle","aux1","aux2"};
    steering_angle_msg.position = {rad2deg * gma_, 0.0, 0.0};
    steering_angle_msg.velocity = {0.0, 0.0, 0.0};
    steering_angle_msg.effort = {0.0, 0.0, 0.0};
    
    steering_angle_pub->publish(steering_angle_msg);

    Matrix<double, 3, 3> R_rb = quat_rb.toRotationMatrix();
    Matrix<double, 3, 1> phi_rb = so3::R2rpy(R_rb);
    psi_gps_r = phi_rb(2, 0);
    R_fb = quat_rb.toRotationMatrix();
    Matrix<double, 3, 1> phi_fb = so3::R2rpy(R_fb);
    psi_gps_f = phi_fb(2, 0);

    Matrix<double, 3, 1> p_gps_f;
    p_gps_f = p_fb + R_fb * p_sagps_f;
    x_gps_f = p_gps_f(0, 0), y_gps_f = p_gps_f(1, 0);
    Matrix<double, 3, 1> p_gps_r;
    p_gps_r = p_rb + R_rb * p_sagps_r;
    x_gps_r = p_gps_r(0, 0), y_gps_r = p_gps_r(1, 0);

    auto gps_r_XYPos = wheel_loader_msgs::msg::XYPos();
    gps_r_XYPos.header.frame_id = "/map";
    gps_r_XYPos.header.stamp = sensor_stamp_;
    gps_r_XYPos.x = x_gps_r;
    gps_r_XYPos.y = y_gps_r;
    gps_r_XYPos_pub->publish(gps_r_XYPos);

    auto gps_f_XYPos = wheel_loader_msgs::msg::XYPos();
    gps_f_XYPos.header.frame_id = "/map";
    gps_f_XYPos.header.stamp = sensor_stamp_;
    gps_f_XYPos.x = x_gps_f;
    gps_f_XYPos.y = y_gps_f;
    gps_f_XYPos_pub->publish(gps_f_XYPos);

    // std::cout << "x_gps_f: " << x_gps_f << " y_gps_f: " << y_gps_f << " psi_gps_f: " << psi_gps_f << std::endl;
    // std::cout << "x_gps_r: " << x_gps_r << " y_gps_r: " << y_gps_r << " psi_gps_r: " << psi_gps_r << std::endl;

    
}