#ifndef LOADER_SIM_HPP
#define LOADER_SIM_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <random>

/*
topic headers
*/
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "wheel_loader_msgs/msg/xy_pos.hpp"

/*
 namespaces
 */
using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace Eigen;

/*
 define the constant values
 */
#define PI 3.141592
#define rad2deg 180 / 3.141592
#define deg2rad 3.141592 / 180

class LoaderSim : public rclcpp::Node
{
public:
    LoaderSim();

    // utility functions
    void compute();
    void topic_publisher();

    // publishing visualization topics
    void visualization();

private:
    /*
     ROS2 settings
     */
    size_t count_;
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;

    /*
     subscribing topics (Msg type, callback function and the recipient variable(s))
     from the driving/operation trajectory planner
     */
    // desired longitudianl accleration of the front body [m/s]
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr front_body_long_acc_sub;
    void front_body_long_acc_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
    double a_f;
    // desired angular rate of the steering angle [deg/s]
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr steering_angle_rate_sub;
    void steering_angle_rate_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
    double steering_angle_rate;
    // desired angular rate of the boom link [deg/s]
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr boom_link_angle_rate_sub;
    void boom_link_angle_rate_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
    double boom_link_angle_rate;
    // desired angular rate of the bucket [deg/s]
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr bucket_angle_rate_sub;
    void bucket_angle_rate_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
    double bucket_angle_rate;

    /*
     publishing topics (Msg type, output variables)
     for the realization of sensor outputs
     */
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr steering_angle_pub;
    double steering_angle; // steering angle [deg]
    rclcpp::Publisher<wheel_loader_msgs::msg::XYPos>::SharedPtr gps_r_XYPos_pub;
    double x_gps_r, y_gps_r;
    double psi_gps_r;
    rclcpp::Publisher<wheel_loader_msgs::msg::XYPos>::SharedPtr gps_f_XYPos_pub;
    double x_gps_f, y_gps_f;
    double psi_gps_f;

    /*
     publishing topics (Msg type, output variable(s))
     for the visualization
     */
    // Pose of the rear body
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rear_body_marker_pub;
    Matrix<double, 3, 1> p_rb;
    Quaternion<double> quat_rb;
    // Pose of the front body
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr front_body_marker_pub;
    Matrix<double, 3, 1> p_fb;
    Quaternion<double> quat_fb;
    // Pose of the boom link
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr boom_marker_pub;
    Matrix<double, 3, 1> p_boom;
    Quaternion<double> quat_boom;
    // Pose of the bucket
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bucket_marker_pub;
    Matrix<double, 3, 1> p_bucket;
    Quaternion<double> quat_bucket;
    // Pose of the bell crank
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bellcrank_marker_pub;
    Matrix<double, 3, 1> p_bellcrank;
    Quaternion<double> quat_bellcrank;
    // Pose of the link
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr link_marker_pub;
    Matrix<double, 3, 1> p_link;
    Quaternion<double> quat_link;
    // Poses of the bucket cylinders
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bkcyl1_marker_pub;
    Matrix<double, 3, 1> p_bkcyl1;
    Quaternion<double> quat_bkcyl1;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bkcyl2_marker_pub;
    Matrix<double, 3, 1> p_bkcyl2;
    Quaternion<double> quat_bkcyl2;
    // Poses of the boom link cylinders
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bcyl1_marker_pub;
    Matrix<double, 3, 1> p_bcyl1;
    Quaternion<double> quat_bcyl1;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bcyl2_marker_pub;
    Matrix<double, 3, 1> p_bcyl2;
    Quaternion<double> quat_bcyl2;

    /*
     parameters
     */
    double gma_0 = 0.0;                        // initial value of the steering angle [in radian]
    double th_b0 = 0.0;                        // initial value of the boom angle [in radian]
    double th_k_pre0 = 0.0;                    // initial value of the (bucket - boom) angle [in radian]
    double a_f_m, a_f_M;                       // minimum and maximum values of the longitudinal speed [in m/s]
    double tau_ = 0.10;                        // time-delay of acceleration tracking 
    Matrix<double, 3, 1> p_sagps_r;            // steering axis to the rear gps
    Matrix<double, 3, 1> p_sagps_f;            // steering axis to the front gps
    double steering_angle_m, steering_angle_M; // minimum and maximum values of the steering angle [in deg]
    Matrix<double, 3, 1> p_rbsa;               // {rb} to steering axis
    Matrix<double, 3, 1> p_safb;               // steering axis to {fb}

    /*
    parameters for HL960SA_00_1-HL960-9SA (wheel loader model)
     */
    Matrix<double, 3, 1> p_fbO;                  // {fb} to axis O
    Matrix<double, 3, 1> p_Oboom;                // axis O to {boom}
    Matrix<double, 3, 1> p_boomH;                // {boom} to axis H
    Matrix<double, 3, 1> p_Hbucket;              // axis H to {bucket}
    double L_1;                                  // horizontal length from the center-of-the-rear-tire to steering axis
    double l_a;
    double L_2;                                  // horizontal length from the steering axis to center-of-the-front-tire
    double L01, L02, L1, L2, L3, L5, L6, L7, L8; // Lengths [in m] of the operational part (Refer to HCE_loader_operational_part_design.png)
    double th_01, th_02, th_c1, th_c2, th_c4;    // Angles [in rad] of the operational part (Refer to HCE_loader_operational_part_design.png

    /*
     variables that is utilized for the whole loop
     */
    double t_0 = 0.0;          // initial ROS time
    double t_old = 0.0;        // the time instant of the previous loop
    double DT = 0.0;           // time interval
    Matrix<double, 3, 3> R_fb; // SO(3) rotation matrix of the front body
    double gma_ = 0.0;         // sterring angle [rad]
    double th_b = 0.0;         // boom link angle [rad]
    double th_k_pre = 0.0;     // (boom - bucket) angle [rad]
    double th_ = 0.0;          // heading angle of the front body [rad]
    double u_f = 0.0;
    double du_f = 0.0;

    // Set up random number generator
    
    
};

#endif