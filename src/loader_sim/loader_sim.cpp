#include "loader_sim/loader_sim.hpp"
#include "so3_utils.hpp"

LoaderSim::LoaderSim() : Node("loader_sim")
{
  timer_ = this->create_wall_timer(
      100ms, std::bind(&LoaderSim::timer_callback, this));
  /*
  topics & services
  */
  // subscribe topics
  front_body_long_acc_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "/loader_sim/front_body/longitudinal_acceleration", 10, std::bind(&LoaderSim::front_body_long_acc_cb, this, _1));
  steering_angle_rate_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "/steering_angle/velocity", 10, std::bind(&LoaderSim::steering_angle_rate_cb, this, _1));
  boom_link_angle_rate_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "/boom_link_angle/velocity", 10, std::bind(&LoaderSim::boom_link_angle_rate_cb, this, _1));
  bucket_angle_rate_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "/bucket_angle/velocity", 10, std::bind(&LoaderSim::bucket_angle_rate_cb, this, _1));

  // publishing topics
  steering_angle_pub = this->create_publisher<sensor_msgs::msg::JointState>(
    "/wheel_loader/steering_angle/position", 10);
  gps_r_XYPos_pub = this->create_publisher<wheel_loader_msgs::msg::XYPos>(
    "/gps/xy_position/rear", 10);
  gps_f_XYPos_pub = this->create_publisher<wheel_loader_msgs::msg::XYPos>(
    "/gps/xy_position/front", 10);

  // marker publishing topics
  rear_body_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(
      "/loader_sim/rear_body/marker", 10);
  front_body_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(
      "/loader_sim/front_body/marker", 10);
  boom_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(
      "/loader_sim/boom/marker", 10);
  bucket_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/loader_sim/bucket/marker", 10);
  bellcrank_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/loader_sim/bellcrank/marker", 10);
  link_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/loader_sim/link/marker", 10);
  bkcyl1_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/loader_sim/bkcyl1/marker", 10);
  bkcyl2_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/loader_sim/bkcyl2/marker", 10);
  bcyl1_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/loader_sim/bcyl1/marker", 10);
  bcyl2_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/loader_sim/bcyl2/marker", 10);

  /*
  parameters
   */
  a_f_m = -0.5;
  a_f_M = 0.5;
  steering_angle_m = -30.0;
  steering_angle_M = 30.0;

  p_rbsa << 0.0, 0.0, 0.0;
  p_safb << 0.0, 0.0, 0.0;
  p_fbO << 0.853, 0.0, 1.365;
  p_Oboom << 0.0, 0.0, 0.0;
  p_boomH << 2.9, 0.0, 0.0;
  p_Hbucket << 0.0, 0.0, 0.0;

  L_1 = 1.650;
  L_2 = 1.650;
  l_a = L_1/2.0;

  L01 = sqrt(0.36507660 * 0.36507660 + 0.05993572 * 0.05993572);
  L02 = sqrt(0.080 * 0.080 + 0.620 * 0.620);
  L1 = sqrt(0.480 * 0.480 + 1.870 * 1.870);
  L2 = 1.550;
  L3 = 2.900000;
  L5 = sqrt(0.05 * 0.05 + 0.40 * 0.40);
  L6 = sqrt(0.2500 * 0.2500 + 0.75993421 * 0.75993421);
  L7 = 0.650000;
  L8 = 0.710;

  th_01 = atan(59.93572 / 365.07660);
  th_02 = atan(80.0 / 620.0);
  th_c1 = atan(480.00000 / 1870.00000);
  th_c2 = 0.0;
  th_c4 = atan(50.0 / 400.0);

  this->declare_parameter("tau_", 0.10);
  tau_ = this->get_parameter("tau_").as_double();
  this->declare_parameter("p_sagps_r_x", 0.0);
  p_sagps_r(0, 0) = this->get_parameter("p_sagps_r_x").as_double();
  this->declare_parameter("p_sagps_r_y", 0.0);
  p_sagps_r(1, 0) = this->get_parameter("p_sagps_r_y").as_double();
  this->declare_parameter("p_sagps_r_z", 0.0);
  p_sagps_r(2, 0) = this->get_parameter("p_sagps_r_z").as_double();
  this->declare_parameter("p_sagps_f_x", 0.0);
  p_sagps_f(0, 0) = this->get_parameter("p_sagps_f_x").as_double();
  this->declare_parameter("p_sagps_f_y", 0.0);
  p_sagps_f(1, 0) = this->get_parameter("p_sagps_f_y").as_double();
  this->declare_parameter("p_sagps_f_z", 0.0);
  p_sagps_f(2, 0) = this->get_parameter("p_sagps_f_z").as_double();

  /*
   input initialization
   */
  a_f = 0.0;
  steering_angle_rate = 0.0;
  boom_link_angle_rate = 0.0;
  bucket_angle_rate = 0.0;

  /*
   state initialization
   */
  u_f = 0.0;
  du_f = 0.0;
  p_fb << L_1, 0.0, 0.0;
  th_ = 0.0;
  gma_ = gma_0;
  th_b = th_b0;
  th_k_pre = th_k_pre0;

  /*
   store the initial ROS time instant
   */
  t_0 = this->now().seconds();
}

/*
callback functions
*/
void LoaderSim::front_body_long_acc_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  a_f = msg->effort[0];
}
void LoaderSim::steering_angle_rate_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  steering_angle_rate = msg->velocity[0];
}
void LoaderSim::boom_link_angle_rate_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  boom_link_angle_rate = msg->velocity[0];
}
void LoaderSim::bucket_angle_rate_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  bucket_angle_rate = msg->velocity[0];
}

void LoaderSim::compute()
{
  // driving kinematics
  double dgma_ = deg2rad * steering_angle_rate;
  
  // input saturation
  a_f = so3::constrain(a_f, a_f_m, a_f_M);
  if (gma_ <= deg2rad * steering_angle_m)
    dgma_ = deg2rad * steering_angle_m - gma_;
  else if (deg2rad * steering_angle_M <= gma_)
    dgma_ = deg2rad * steering_angle_M - gma_;

  // kinematics update
  // std::cout << "X_: " << X_.transpose() << std::endl;
  std::random_device rd; // Obtain a seed from the hardware
  std::default_random_engine generator(rd()); // Initialize the generator with the seed

  // Set up the normal distribution with mean 0.0 and standard deviation 5.0
  std::normal_distribution<double> distribution(0.0, 5.0);
  double aph_f, aph_r; // slip angles (front, tire)
  aph_f = deg2rad * distribution(generator);
  aph_r = deg2rad * distribution(generator);

  // std::cout << "aph_f: " << aph_f << " aph_r: " << aph_r << std::endl;
  
  double dth_ = L_2 * dgma_ / (L_1 + L_2) + (aph_r - aph_f)*u_f/(L_1 + L_2);
  double v_f = - aph_f * u_f - l_a * dth_;

  Matrix<double, 3, 1> dp_fb;
  dp_fb(0) = u_f * cos(th_) - v_f * sin(th_);
  dp_fb(1) = u_f * sin(th_) + v_f * cos(th_);
  dp_fb(2) = 0.0;

  double a_x = du_f + l_a * dth_ * dth_;
  double ddu_f = (1.0/tau_) * (-a_x + a_f);

  th_ = th_ + dth_ * DT;
  gma_ = gma_ + dgma_ * DT;
  u_f = u_f + du_f * DT;
  du_f = du_f + ddu_f * DT;
  p_fb = p_fb + dp_fb * DT;

  // std::cout << "p_fb_: " << p_fb.transpose() << std::endl;
  R_fb = so3::rpy2R(0.0, 0.0, th_);

  /*
   operation part
   */
  th_b = th_b + deg2rad * boom_link_angle_rate * DT;
  th_k_pre = th_k_pre + deg2rad * bucket_angle_rate * DT;
}

// execution
void LoaderSim::timer_callback()
{
  double t_cur = this->now().seconds() - t_0; // time calculation
  DT = t_cur - t_old;                         // time interval calculation

  /*
   driving check
   */
  a_f = 0.01;
  steering_angle_rate = deg2rad*0.0;

  /*
   operation part check
   */
  boom_link_angle_rate = 15.0 * sin(2.0*PI*t_cur/5.0);
  bucket_angle_rate = 15.0 * cos(2.0*PI*t_cur/5.0);

  this->compute();
  this->visualization();

  this->topic_publisher();

  t_old = t_cur;
}