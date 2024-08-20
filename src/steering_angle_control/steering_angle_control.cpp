#include "steering_angle_control/steering_angle_control.hpp"
#include "so3_utils.hpp"

SteeringAngleControl::SteeringAngleControl() : Node("steering_angle_control")
{
    timer_ = this->create_wall_timer(
        10ms, std::bind(&SteeringAngleControl::timer_callback, this));
    /*
    topics & services
    */
    // subscribe topics
    steering_rate_sp_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "/wheel_loader/steering_rate/setpoint", 10, std::bind(&SteeringAngleControl::steering_rate_sp_cb, this, _1));
    from_loader_sub = this->create_subscription<wheel_loader_msgs::msg::LoaderTx>(
        "/wheel_loader/tx", 10, std::bind(&SteeringAngleControl::from_loader_cb, this, _1));

    // publishing topics
    to_loader_pub = this->create_publisher<wheel_loader_msgs::msg::LoaderRx>(
        "/wheel_loader/rx", 10);

    /*
    parameters
     */
    this->declare_parameter("sim_flag", false);
    sim_flag = this->get_parameter("sim_flag").as_bool();
    this->declare_parameter("tau_sim", 0.20);
    tau_sim = this->get_parameter("tau_sim").as_double();

    this->declare_parameter("k_p", 0.05);
    k_p = this->get_parameter("k_p").as_double();
    this->declare_parameter("k_i", 0.0);
    k_i = this->get_parameter("k_i").as_double();
    this->declare_parameter("k_d", 0.01);
    k_d = this->get_parameter("k_d").as_double();

    this->declare_parameter("u_m", -0.80);
    u_m = this->get_parameter("u_m").as_double();
    this->declare_parameter("u_M", 0.80);
    u_M = this->get_parameter("u_M").as_double();

    this->declare_parameter("steering_rate_m", -20.0);
    steering_rate_m = this->get_parameter("steering_rate_m").as_double();
    this->declare_parameter("steering_rate_M", 20.0);
    steering_rate_M = this->get_parameter("steering_rate_M").as_double();

    this->declare_parameter("a_u_to_valve", 5000.0);
    a_u_to_valve = this->get_parameter("a_u_to_valve").as_double();
    this->declare_parameter("b_u_to_valve", 5000.0);
    b_u_to_valve = this->get_parameter("b_u_to_valve").as_double();
    this->declare_parameter("target_electric_steering_actuation_control_mode", 0);
    steering_valve_con_mode = this->get_parameter("target_electric_steering_actuation_control_mode").as_int();

    // input initialization
    dgma_d = 0.0;

    /*
     output initialization
     */
    u_ = 0.0;

    /*
     state initialization
     */
    est_dgma_ = 0.0;
    e_i = 0.0;
    gma_d = 0.0;
    P_gma.setIdentity();
    dgma_ = 0.0;

    /*
     store the initial ROS time instant
     */
    t_0 = this->now().seconds();
}

/*
callback functions
*/
void SteeringAngleControl::from_loader_cb(const wheel_loader_msgs::msg::LoaderTx::SharedPtr msg)
{
    gma_ = deg2rad * (msg->boom_angle);
}
void SteeringAngleControl::steering_rate_sp_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    dgma_d = deg2rad * (msg->velocity[0]);
}

// execution
void SteeringAngleControl::timer_callback()
{
    double t_cur = this->now().seconds() - t_0; // time calculation
    DT = t_cur - t_old;                         // time interval calculation

    // virtual inputs
    if (sim_flag)
    {
        double gma_t = deg2rad * 20.0;
        dgma_d = gma_t - gma_;
    }

    // steering rate calculation (kalman filter)
    Matrix<double, 2, 2> F_; // propagation matrix
    F_ << 1.0, DT,
        0.0, 1.0;
    Matrix<double, 2, 1> N_; // process noise matrix
    N_ << 0.0,
        DT;
    Matrix<double, 1, 2> H_; // output matrix
    H_ << 1.0, 0.0;

    /*
     prediction
     */
    double q_ = 15.0;        // process noise covariance
    Matrix<double, 2, 1> z_; // priori of state estimation
    Matrix<double, 2, 2> M_; // priori of state convariance
    Matrix<double, 2, 1> est_Gma_;
    est_Gma_ << est_gma_, est_dgma_;
    z_ = F_ * est_Gma_;
    M_ = F_ * P_gma * F_.transpose() + N_ * q_ * q_ * N_.transpose();

    /*
     update
     */
    double r_; // measurement noise covariance
    r_ = 0.05;
    Matrix<double, 2, 1> K_; // Kalman gain
    K_ = M_ * H_.transpose() / (H_ * M_ * H_.transpose() + r_);
    est_Gma_ = z_ + K_ * (gma_ - H_ * z_);
    P_gma = M_ - K_ * H_ * M_;

    est_gma_ = gma_;
    est_dgma_ = est_Gma_(1, 0);

    // PID control
    e_p = gma_d - gma_;
    e_i = e_i + e_p * DT;
    e_d = dgma_d - est_dgma_;

    u_ = rad2deg * (k_p * e_p + k_i * e_i + k_d * e_d);
    u_ = so3::constrain(u_, u_m, u_M);

    double steering_valve_control = a_u_to_valve * u_ + b_u_to_valve;

    builtin_interfaces::msg::Time sensor_stamp_ = this->now();

    // publish the steering valve
    auto to_loader_msg = wheel_loader_msgs::msg::LoaderRx();

    to_loader_msg.header.frame_id = "";
    to_loader_msg.header.stamp = sensor_stamp_;

    to_loader_msg.target_electric_steering_actuation = steering_valve_control;
    to_loader_msg.target_electric_steering_actuation_control_mode = steering_valve_con_mode;

    to_loader_pub->publish(to_loader_msg);

    // desired steering angle calculation
    gma_d = gma_d + dgma_d * DT;

    // virtual dynamics
    if (sim_flag)
    {
        double A_, B_;
        A_ = 23.444, B_ = 0.44321;

        double dgma_input = A_ * tanh(u_ / B_);
        dgma_ = tau_sim * dgma_input + (1.0 - tau_sim) * dgma_;

        gma_ = gma_ + dgma_ * DT;

        std::cout << "gma_d: " << rad2deg * gma_d << "[deg], gma_: " << rad2deg * gma_ << " [deg]" << std::endl;
    }

    t_old = t_cur;
}