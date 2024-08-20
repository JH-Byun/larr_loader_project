#ifndef STEERING_ANGLE_CONTROL_HPP
#define STEERING_ANGLE_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <Eigen/Dense>

/*
topic headers
*/
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "wheel_loader_msgs/msg/loader_tx.hpp"
#include "wheel_loader_msgs/msg/loader_rx.hpp"

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

class SteeringAngleControl : public rclcpp::Node
{
public:
    SteeringAngleControl();

    // utility functions

private:
    /*
     ROS2 settings
     */
    size_t count_;
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;

    /*
     From Loader:
     1. Header
     2. Boom angle [deg]
     3. Bucket cylinder length [mm]
     4. Steering angle [deg]
     5. Wheel-based vehicle speed [kph]
     */
    rclcpp::Subscription<wheel_loader_msgs::msg::LoaderTx>::SharedPtr from_loader_sub;
    void from_loader_cb(const wheel_loader_msgs::msg::LoaderTx::SharedPtr msg);
    double gma_; // current steering angle [deg]
    
    // desired steering angle rate [deg/s]
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr steering_rate_sp_sub;
    void steering_rate_sp_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
    double dgma_d; // [rad/s]

    /*
     To Loader:
     1. Header
     4. Target electric steering actuation (-10000 % ~ 10000 %)
     */
    rclcpp::Publisher<wheel_loader_msgs::msg::LoaderRx>::SharedPtr to_loader_pub;
    double u_; // normalized steering valve control [-1.0 ~ 1.0]

    /*
     parameters
     */
    bool sim_flag;                           // simulation flag
    double k_p, k_i, k_d;                    // PID gains for the steering rate control
    double u_m, u_M;                         // minimum and maximum values of the steering valve control input [-]
    double a_u_to_valve, b_u_to_valve;       // normalization factor
    uint32_t steering_valve_con_mode = 0;    // steering valve control mode
    double steering_rate_m, steering_rate_M; // minimum and maximum values of the steering rate [deg/s]
    double tau_sim;

    /*
     variables that is utilized for the whole loop
     */
    double t_0 = 0.0;       // initial ROS time
    double t_old = 0.0;     // the time instant of the previous loop
    double DT = 0.0;        // time interval
    double est_gma_ = 0.0;  // steering angle [rad]
    double est_dgma_ = 0.0; // steering rate [rad/s]
    double gma_d = 0.0;     // steering angle setpoint [rad]
    double e_p, e_i, e_d;   // errors [rad]
    Matrix<double, 2, 2> P_gma;

    double dgma_; // steering rate for simulation [rad/s]

    // Set up random number generator
};

#endif