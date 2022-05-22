#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <jsk_recognition_utils/geo_util.h>
#include <sensor_msgs/Imu.h>
#include <safe_footstep_planner/OnlineFootStep.h>
#include <vector>

class OdomPublisher
{
public:
  OdomPublisher();
  ~OdomPublisher(){};
private:
  tf::TransformListener tfl;
  Eigen::Affine3f body_to_leg_transform;
  Eigen::Affine3f leg_to_imu_transform;
  Eigen::Affine3f body_to_groundleg_transform;
  Eigen::Affine3f groundleg_to_odom_transform;
  tf::TransformBroadcaster br;
  ros::NodeHandle nh;
  ros::Subscriber sub;
  std::string leg_frame;
  std::string prev_leg_frame;
  int change_leg_counter;
  bool init_flag;
  ros::Time begin_time;
  ros::Time a_time;
  ros::Time b_time;
  ros::Time c_time;
  ros::Time d_time;
  ros::Time e_time;
  ros::Time f_time;
  ros::Time end_time;

  std::vector<ros::Time> stamps;

  ros::Rate r;

  void cb(const safe_footstep_planner::OnlineFootStep::ConstPtr& msg);
  void pubodom();
};

OdomPublisher::OdomPublisher() : nh(""), r(50)
{
  sub = nh.subscribe("landing_target", 1, &OdomPublisher::cb, this);
  leg_frame = "rleg_end_coords";
  prev_leg_frame = "lleg_end_coords";
  change_leg_counter = -1;
  init_flag = true;
  ros::Time tmp = ros::Time::now();
  stamps = std::vector<ros::Time>{tmp, tmp, tmp, tmp};
  groundleg_to_odom_transform.setIdentity();
  pubodom();
}

void OdomPublisher::pubodom()
{
  while (nh.ok()) {
    for (int i = 0; i < stamps.size()-1; i++) {
      stamps[i] = stamps[i+1];
    }
    stamps[stamps.size()-1] = ros::Time::now();
    if (change_leg_counter == 0) {
      std::string tmp = prev_leg_frame;
      prev_leg_frame = leg_frame;
      leg_frame = tmp;
    }
    try{
      tf::StampedTransform transform;
      tfl.waitForTransform("BODY", leg_frame, stamps[0], ros::Duration(1.0));
      tfl.lookupTransform("BODY", leg_frame, stamps[0], transform);
      tf::transformTFToEigen(transform, body_to_leg_transform);

      tfl.waitForTransform(leg_frame, "imu_floor", stamps[0], ros::Duration(1.0));
      tfl.lookupTransform(leg_frame, "imu_floor", stamps[0], transform);
      tf::transformTFToEigen(transform, leg_to_imu_transform);

      Eigen::Vector3f ez = leg_to_imu_transform.rotation() * Eigen::Vector3f::UnitZ();
      Eigen::Vector3f ex = Eigen::Vector3f::UnitX();
      Eigen::Vector3f ey = ez.cross(ex).normalized();
      ex = ey.cross(ez).normalized();

      Eigen::Matrix3f m;
      m << ex, ey, ez;
      Eigen::Affine3f leg_to_groundleg_transform = Eigen::Translation<float, 3>(0, 0, 0) * m;
      body_to_groundleg_transform = body_to_leg_transform * leg_to_groundleg_transform;

      if (change_leg_counter == 0) {
        Eigen::Affine3f body_to_prevleg_transform;
        tfl.waitForTransform("BODY", prev_leg_frame, stamps[0], ros::Duration(1.0));
        tfl.lookupTransform("BODY", prev_leg_frame, stamps[0], transform);
        tf::transformTFToEigen(transform, body_to_prevleg_transform);

        ez = (body_to_prevleg_transform.inverse() * body_to_leg_transform * leg_to_imu_transform).rotation() * Eigen::Vector3f::UnitZ();
        ex = Eigen::Vector3f::UnitX();
        ey = ez.cross(ex).normalized();
        ex = ey.cross(ez).normalized();

        m = Eigen::Matrix3f::Identity();
        m << ex, ey, ez;
        Eigen::Affine3f prevleg_to_groundprevleg_transform = Eigen::Translation<float, 3>(0, 0, 0) * m;
        Eigen::Affine3f body_to_groundprevleg_transform = body_to_prevleg_transform * prevleg_to_groundprevleg_transform;

        groundleg_to_odom_transform = body_to_groundleg_transform.inverse() * body_to_groundprevleg_transform * groundleg_to_odom_transform;

        ez = (leg_to_groundleg_transform.inverse() * leg_to_imu_transform).rotation() * Eigen::Vector3f::UnitZ();
        ex = groundleg_to_odom_transform.rotation() * Eigen::Vector3f::UnitX();
        ey = ez.cross(ex).normalized();
        ex = ey.cross(ez).normalized();
        m = Eigen::Matrix3f::Identity();
        m << ex, ey, ez;
        groundleg_to_odom_transform = Eigen::Translation<float, 3>(groundleg_to_odom_transform(0, 3), groundleg_to_odom_transform(1, 3), groundleg_to_odom_transform(2, 3)) * m;
        //Eigen::Affine3f tmp = Eigen::Translation<float, 3>(0, 0, 0) * m;
        //groundleg_to_odom_transform = tmp * groundleg_to_odom_transform;
      }

      Eigen::Affine3f body_to_odom_transform = body_to_groundleg_transform * groundleg_to_odom_transform;

      tf::Transform output_transform;
      tf::transformEigenToTF(body_to_odom_transform, output_transform);
      br.sendTransform(tf::StampedTransform(output_transform, stamps[0], "BODY", "leg_base_odom"));

    } catch (tf::TransformException ex){
      std::cout << "error" << std::endl;
    }
    if (change_leg_counter >= 0) change_leg_counter--;
    ros::spinOnce();
    r.sleep();
    end_time = ros::Time::now();
    //std::cout << "all_time " << (end_time - begin_time).sec << "s " << (int)((end_time - begin_time).nsec / 1000000) << "ms" << std::endl;
    //std::cout << "begin_a  " << (a_time - begin_time).sec << "s " << (int)((a_time - begin_time).nsec / 1000000) << "ms" << std::endl;
    //std::cout << "a_b  " << (b_time - a_time).sec << "s " << (int)((b_time - a_time).nsec / 1000000) << "ms" << std::endl;
    //std::cout << "b_c  " << (c_time - b_time).sec << "s " << (int)((c_time - b_time).nsec / 1000000) << "ms" << std::endl;
    //std::cout << "c_d  " << (d_time - c_time).sec << "s " << (int)((d_time - c_time).nsec / 1000000) << "ms" << std::endl;
    //std::cout << "d_e  " << (e_time - d_time).sec << "s " << (int)((e_time - d_time).nsec / 1000000) << "ms" << std::endl;
    //std::cout << "e_f  " << (f_time - e_time).sec << "s " << (int)((f_time - e_time).nsec / 1000000) << "ms" << std::endl;
    //std::cout << "f_end  " << (end_time - f_time).sec << "s " << (int)((end_time - f_time).nsec / 1000000) << "ms" << std::endl;
  }
}

void OdomPublisher::cb(const safe_footstep_planner::OnlineFootStep::ConstPtr& msg)
{
  if (init_flag) {
    leg_frame = (msg->l_r == 0 ? "rleg_end_coords" : "lleg_end_coords");
    prev_leg_frame = (msg->l_r == 0 ? "lleg_end_coords" : "rleg_end_coords");
    init_flag = false;
    return;
  }
  if (msg->l_r == 0 && leg_frame == "lleg_end_coords" && change_leg_counter < 0) {
    change_leg_counter = 5;
  } else if (msg->l_r == 1 && leg_frame == "rleg_end_coords" && change_leg_counter < 0) {
    change_leg_counter = 5;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_odom_publisher");
  OdomPublisher odom_publisher;
  return 0;
}
