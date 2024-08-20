#include "loader_sim/loader_sim.hpp"
#include "so3_utils.hpp"

void LoaderSim::visualization()
{
    /*
     front body pose calculation
     */
    quat_fb = so3::R2q(R_fb);

    /*
     rear body pose calculation
     */
    Matrix<double, 3, 3> R_rb = R_fb * so3::rpy2R(0.0, 0.0, -gma_);
    quat_rb = so3::R2q(R_rb);
    p_rb = p_fb;

    /*
     boom link pose calculation
     */
    Matrix<double, 3, 3> R_boom = R_fb * (so3::rpy2R(0.0, -th_b, 0.0));
    quat_boom = so3::R2q(R_boom);
    p_boom = p_fb + R_fb * p_fbO + R_boom * p_Oboom;

    /*
     bucket pose calculation
     */
    Matrix<double, 3, 3> R_bucket = R_boom * (so3::rpy2R(0.0, -th_k_pre, 0.0));
    quat_bucket = so3::R2q(R_bucket);
    p_bucket = p_boom + R_boom * p_boomH + R_bucket * p_Hbucket;

    /*
     bell crank pose calculation
     */
    double th_1 = PI / 2.0 - (th_c4 + th_k_pre);
    double a_ = L3 - L5 * cos(th_1) - L1 * cos(th_c1);
    double b_ = L5 * sin(th_1) - L1 * sin(th_c1);
    double c_ = (0.5 / L6) * (a_ * a_ + b_ * b_ + L6 * L6 - L8 * L8);
    double th_2 = acos((1.0 / (a_ * a_ + b_ * b_)) * (a_ * c_ - sqrt(b_ * b_ * (a_ * a_ + b_ * b_ - c_ * c_))));
    double th_bell_pre = PI - th_2 + (atan(778.97752 / 5.91746) - atan(758.01323 / 255.76542));
    double th_bell = th_bell_pre + th_b;
    Matrix<double, 3, 1> pboom_OE;
    pboom_OE << L1 * cos(th_c1), 0.0, L1 * sin(th_c1);
    Matrix<double, 3, 3> R_bellcrank = R_fb * (so3::rpy2R(0.0, -(th_bell - PI / 2.0), 0.0));
    quat_bellcrank = so3::R2q(R_bellcrank);
    Matrix<double, 3, 1> p_O = p_fb + R_fb * p_fbO;
    p_bellcrank = p_O + R_boom * pboom_OE;

    /*
     link pose calculation
     */
    double th_3 = atan((L5 * sin(th_1) - L1 * sin(th_c1) + L6 * sin(th_2)) / (L3 - L5 * cos(th_1) - L1 * cos(th_c1) - L6 * cos(th_2)));
    Matrix<double, 3, 3> R_link = R_fb * (so3::rpy2R(0.0, -(th_3 + th_b), 0.0));
    quat_link = so3::R2q(R_link);
    Matrix<double, 3, 1> pboom_HG;
    pboom_HG << -L5 * cos(th_1), 0.0, L5 * sin(th_1);
    p_link = p_bucket + R_boom * pboom_HG;

    /*
     BK_cyl_1, BK_cyl_2 poses calculation
     */
    Matrix<double, 3, 1> pfb_OB, pfb_OE, pfb_EC;
    pfb_OB << L01 * cos(th_01), 0.0, -L01 * sin(th_01);
    pfb_OE << L1 * cos(th_c1 + th_b), 0.0, L1 * sin(th_c1 + th_b);
    pfb_EC << L7 * cos(th_bell), 0.0, L7 * sin(th_bell);
    Matrix<double, 3, 1> pfb_BC = -pfb_OB + pfb_OE + pfb_EC;
    double th_X_k = atan(pfb_BC(2) / pfb_BC(0));
    Matrix<double, 3, 3> R_bkcyl1 = R_fb * (so3::rpy2R(0.0, -th_X_k, 0.0));
    Quaternion<double> quat_bkcyl1 = so3::R2q(R_bkcyl1);
    Matrix<double, 3, 3> R_bkcyl2 = R_fb * (so3::rpy2R(0.0, -th_X_k, 0.0));
    quat_bkcyl2 = so3::R2q(R_bkcyl2);
    p_bkcyl1 = p_boom + R_fb * pfb_OB;
    p_bkcyl2 = p_bkcyl1 + R_fb * pfb_BC;

    /*
     B_cyl_1, B_cyl_2 poses calcuation
     */
    Matrix<double, 3, 1> pfb_OA, pfb_OD;
    pfb_OA << -L02 * sin(th_02), 0.0, -L02 * cos(th_02);
    pfb_OD << L2 * cos(th_b - th_c2), 0.0, L2 * sin(th_b - th_c2);
    Matrix<double, 3, 1> pfb_AD = -pfb_OA + pfb_OD;
    double th_X_b = atan(pfb_AD(2) / pfb_AD(0));
    Matrix<double, 3, 3> R_bcyl1 = R_fb * (so3::rpy2R(0.0, -th_X_b, 0.0));
    quat_bcyl1 = so3::R2q(R_bcyl1);
    Matrix<double, 3, 3> R_bcyl2 = R_fb * (so3::rpy2R(0.0, -th_X_b, 0.0));
    quat_bcyl2 = so3::R2q(R_bcyl2);
    p_bcyl1 = p_boom + R_fb * pfb_OA;
    p_bcyl2 = p_bcyl1 + R_fb * pfb_AD;

    /*
     set common values for the visualization topics
     */
    builtin_interfaces::msg::Time vis_stamp_ = this->now();
    // set scale
    geometry_msgs::msg::Vector3 scale_;
    scale_.x = 1.0, scale_.y = 1.0, scale_.z = 1.0;
    // set color
    std_msgs::msg::ColorRGBA color_;
    color_.r = 1.0, color_.g = 1.0, color_.b = 1.0, color_.a = 0.5;
    // import stl folder (/mesh)
    std::string package_directory = ament_index_cpp::get_package_share_directory("larr_loader_project");

    // publish rear body's visual marker
    visualization_msgs::msg::Marker rear_body_marker;
    rear_body_marker.header.frame_id = "map";
    rear_body_marker.header.stamp = vis_stamp_;
    rear_body_marker.ns = "rear_body";
    rear_body_marker.id = 0;
    rear_body_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    rear_body_marker.action = visualization_msgs::msg::Marker::ADD;
    rear_body_marker.pose.position.x = p_rb(0);
    rear_body_marker.pose.position.y = p_rb(1);
    rear_body_marker.pose.position.z = p_rb(2);
    rear_body_marker.pose.orientation.w = quat_rb.w();
    rear_body_marker.pose.orientation.x = quat_rb.x();
    rear_body_marker.pose.orientation.y = quat_rb.y();
    rear_body_marker.pose.orientation.z = quat_rb.z();
    rear_body_marker.scale = scale_;
    rear_body_marker.color = color_;
    rear_body_marker.mesh_resource = "package://larr_loader_project/meshes/rear_cabin_jh.stl";
    rear_body_marker_pub->publish(rear_body_marker);

    // publish front body's visual marker
    visualization_msgs::msg::Marker front_body_marker;
    front_body_marker.header.frame_id = "map";
    front_body_marker.header.stamp = vis_stamp_;
    front_body_marker.ns = "front_tire";
    front_body_marker.id = 1;
    front_body_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    front_body_marker.action = visualization_msgs::msg::Marker::ADD;
    front_body_marker.pose.position.x = p_fb(0);
    front_body_marker.pose.position.y = p_fb(1);
    front_body_marker.pose.position.z = p_fb(2);
    front_body_marker.pose.orientation.w = quat_fb.w();
    front_body_marker.pose.orientation.x = quat_fb.x();
    front_body_marker.pose.orientation.y = quat_fb.y();
    front_body_marker.pose.orientation.z = quat_fb.z();
    front_body_marker.scale = scale_;
    front_body_marker.color = color_;
    front_body_marker.mesh_resource = "package://larr_loader_project/meshes/front_tire_jh.stl";
    front_body_marker_pub->publish(front_body_marker);

    // publish boom link's visual marker
    visualization_msgs::msg::Marker boom_marker;
    boom_marker.header.frame_id = "map";
    boom_marker.header.stamp = vis_stamp_;
    boom_marker.ns = "boom";
    boom_marker.id = 2;
    boom_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    boom_marker.action = visualization_msgs::msg::Marker::ADD;
    boom_marker.pose.position.x = p_boom(0);
    boom_marker.pose.position.y = p_boom(1);
    boom_marker.pose.position.z = p_boom(2);
    boom_marker.pose.orientation.w = quat_boom.w();
    boom_marker.pose.orientation.x = quat_boom.x();
    boom_marker.pose.orientation.y = quat_boom.y();
    boom_marker.pose.orientation.z = quat_boom.z();
    boom_marker.scale = scale_;
    boom_marker.color = color_;
    boom_marker.mesh_resource = "package://larr_loader_project/meshes/boom_link_integrated_jh.stl";
    boom_marker_pub->publish(boom_marker);

    // publish bucket's visual marker
    visualization_msgs::msg::Marker bucket_marker;
    bucket_marker.header.frame_id = "map";
    bucket_marker.header.stamp = vis_stamp_;
    bucket_marker.ns = "bucket";
    bucket_marker.id = 3;
    bucket_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    bucket_marker.action = visualization_msgs::msg::Marker::ADD;
    bucket_marker.pose.position.x = p_bucket(0);
    bucket_marker.pose.position.y = p_bucket(1);
    bucket_marker.pose.position.z = p_bucket(2);
    bucket_marker.pose.orientation.w = quat_bucket.w();
    bucket_marker.pose.orientation.x = quat_bucket.x();
    bucket_marker.pose.orientation.y = quat_bucket.y();
    bucket_marker.pose.orientation.z = quat_bucket.z();
    bucket_marker.scale = scale_;
    bucket_marker.color = color_;
    bucket_marker.mesh_resource = "package://larr_loader_project/meshes/bucket_jh.stl";
    bucket_marker_pub->publish(bucket_marker);

    // publish bellcrank's visual marker
    visualization_msgs::msg::Marker bellcrank_marker;
    bellcrank_marker.header.frame_id = "map";
    bellcrank_marker.header.stamp = vis_stamp_;
    bellcrank_marker.ns = "bellcrank";
    bellcrank_marker.id = 4;
    bellcrank_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    bellcrank_marker.action = visualization_msgs::msg::Marker::ADD;
    bellcrank_marker.pose.position.x = p_bellcrank(0);
    bellcrank_marker.pose.position.y = p_bellcrank(1);
    bellcrank_marker.pose.position.z = p_bellcrank(2);
    bellcrank_marker.pose.orientation.w = quat_bellcrank.w();
    bellcrank_marker.pose.orientation.x = quat_bellcrank.x();
    bellcrank_marker.pose.orientation.y = quat_bellcrank.y();
    bellcrank_marker.pose.orientation.z = quat_bellcrank.z();
    bellcrank_marker.scale = scale_;
    bellcrank_marker.color = color_;
    bellcrank_marker.mesh_resource = "package://larr_loader_project/meshes/bellcrank_jh.stl";
    bellcrank_marker_pub->publish(bellcrank_marker);

    // publish link's visual marker
    visualization_msgs::msg::Marker link_marker;
    link_marker.header.frame_id = "map";
    link_marker.header.stamp = vis_stamp_;
    link_marker.ns = "link";
    link_marker.id = 5;
    link_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    link_marker.action = visualization_msgs::msg::Marker::ADD;
    link_marker.pose.position.x = p_link(0);
    link_marker.pose.position.y = p_link(1);
    link_marker.pose.position.z = p_link(2);
    link_marker.pose.orientation.w = quat_link.w();
    link_marker.pose.orientation.x = quat_link.x();
    link_marker.pose.orientation.y = quat_link.y();
    link_marker.pose.orientation.z = quat_link.z();
    link_marker.scale = scale_;
    link_marker.color = color_;
    link_marker.mesh_resource = "package://larr_loader_project/meshes/link_jh.stl";
    link_marker_pub->publish(link_marker);

    // publish the first bucket cylinder's visual marker
    visualization_msgs::msg::Marker bkcyl1_marker;
    bkcyl1_marker.header.frame_id = "map";
    bkcyl1_marker.header.stamp = vis_stamp_;
    bkcyl1_marker.ns = "bucket cylinder 1";
    bkcyl1_marker.id = 6;
    bkcyl1_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    bkcyl1_marker.action = visualization_msgs::msg::Marker::ADD;
    bkcyl1_marker.pose.position.x = p_bkcyl1(0);
    bkcyl1_marker.pose.position.y = p_bkcyl1(1);
    bkcyl1_marker.pose.position.z = p_bkcyl1(2);
    bkcyl1_marker.pose.orientation.w = quat_bkcyl1.w();
    bkcyl1_marker.pose.orientation.x = quat_bkcyl1.x();
    bkcyl1_marker.pose.orientation.y = quat_bkcyl1.y();
    bkcyl1_marker.pose.orientation.z = quat_bkcyl1.z();
    bkcyl1_marker.scale = scale_;
    bkcyl1_marker.color = color_;
    bkcyl1_marker.mesh_resource = "package://larr_loader_project/meshes/bkcyl1_jh.stl";
    bkcyl1_marker_pub->publish(bkcyl1_marker);

    // publish the second bucket cylinder's visual marker
    visualization_msgs::msg::Marker bkcyl2_marker;
    bkcyl2_marker.header.frame_id = "map";
    bkcyl2_marker.header.stamp = vis_stamp_;
    bkcyl2_marker.ns = "bucket cylinder 2";
    bkcyl2_marker.id = 7;
    bkcyl2_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    bkcyl2_marker.action = visualization_msgs::msg::Marker::ADD;
    bkcyl2_marker.pose.position.x = p_bkcyl2(0);
    bkcyl2_marker.pose.position.y = p_bkcyl2(1);
    bkcyl2_marker.pose.position.z = p_bkcyl2(2);
    bkcyl2_marker.pose.orientation.w = quat_bkcyl2.w();
    bkcyl2_marker.pose.orientation.x = quat_bkcyl2.x();
    bkcyl2_marker.pose.orientation.y = quat_bkcyl2.y();
    bkcyl2_marker.pose.orientation.z = quat_bkcyl2.z();
    bkcyl2_marker.scale = scale_;
    bkcyl2_marker.color = color_;
    bkcyl2_marker.mesh_resource = "package://larr_loader_project/meshes/bkcyl2_jh.stl";
    bkcyl2_marker_pub->publish(bkcyl2_marker);

    // publish the first boom link cylinder's visual marker
    visualization_msgs::msg::Marker bcyl1_marker;
    bcyl1_marker.header.frame_id = "map";
    bcyl1_marker.header.stamp = vis_stamp_;
    bcyl1_marker.ns = "boom cylinder 1";
    bcyl1_marker.id = 8;
    bcyl1_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    bcyl1_marker.action = visualization_msgs::msg::Marker::ADD;
    bcyl1_marker.pose.position.x = p_bcyl1(0);
    bcyl1_marker.pose.position.y = p_bcyl1(1);
    bcyl1_marker.pose.position.z = p_bcyl1(2);
    bcyl1_marker.pose.orientation.w = quat_bcyl1.w();
    bcyl1_marker.pose.orientation.x = quat_bcyl1.x();
    bcyl1_marker.pose.orientation.y = quat_bcyl1.y();
    bcyl1_marker.pose.orientation.z = quat_bcyl1.z();
    bcyl1_marker.scale = scale_;
    bcyl1_marker.color = color_;
    bcyl1_marker.mesh_resource = "package://larr_loader_project/meshes/bcyl1_jh.stl";
    bcyl1_marker_pub->publish(bcyl1_marker);

    // publish the second boom link cylinder's visual marker
    visualization_msgs::msg::Marker bcyl2_marker;
    bcyl2_marker.header.frame_id = "map";
    bcyl2_marker.header.stamp = vis_stamp_;
    bcyl2_marker.ns = "boom cylinder 2";
    bcyl2_marker.id = 9;
    bcyl2_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    bcyl2_marker.action = visualization_msgs::msg::Marker::ADD;
    bcyl2_marker.pose.position.x = p_bcyl2(0);
    bcyl2_marker.pose.position.y = p_bcyl2(1);
    bcyl2_marker.pose.position.z = p_bcyl2(2);
    bcyl2_marker.pose.orientation.w = quat_bcyl2.w();
    bcyl2_marker.pose.orientation.x = quat_bcyl2.x();
    bcyl2_marker.pose.orientation.y = quat_bcyl2.y();
    bcyl2_marker.pose.orientation.z = quat_bcyl2.z();
    bcyl2_marker.scale = scale_;
    bcyl2_marker.color = color_;
    bcyl2_marker.mesh_resource = "package://larr_loader_project/meshes/bcyl2_jh.stl";
    bcyl2_marker_pub->publish(bcyl2_marker);
}