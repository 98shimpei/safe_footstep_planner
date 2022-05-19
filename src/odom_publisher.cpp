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
  Eigen::Affine3f leg_transform;
  Eigen::Affine3f odom_transform;
  Eigen::Affine3f imu_transform;
  tf::TransformBroadcaster br;
  ros::NodeHandle nh;
  ros::Subscriber sub;
  std::string leg_frame;
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
  ros::Time tmp = ros::Time::now();
  stamps = std::vector<ros::Time>{tmp, tmp, tmp, tmp};
  pubodom();
}

void OdomPublisher::pubodom()
{
  while (nh.ok()) {
    begin_time = ros::Time::now();
    for (int i = 0; i < stamps.size()-1; i++) {
      stamps[i] = stamps[i+1];
    }
    stamps[stamps.size()-1] = ros::Time::now();
    try{
      tf::StampedTransform transform;
      tfl.waitForTransform("BODY", leg_frame, stamps[0], ros::Duration(1.0));
      tfl.lookupTransform("BODY", leg_frame, stamps[0], transform);
      tf::transformTFToEigen(transform, leg_transform);
      a_time = ros::Time::now();
      tfl.waitForTransform(leg_frame, "odom", stamps[0], ros::Duration(1.0));
      tfl.lookupTransform(leg_frame, "odom", stamps[0], transform);
      tf::transformTFToEigen(transform, odom_transform);
      b_time = ros::Time::now();
      tfl.waitForTransform(leg_frame, "imu_floor", stamps[0], ros::Duration(1.0));
      tfl.lookupTransform(leg_frame, "imu_floor", stamps[0], transform);
      tf::transformTFToEigen(transform, imu_transform);
      c_time = ros::Time::now();
      Eigen::Vector3f ez = imu_transform.rotation() * Eigen::Vector3f::UnitZ();
      Eigen::Vector3f ex = odom_transform.rotation() * Eigen::Vector3f::UnitX();
      Eigen::Vector3f ey = ez.cross(ex).normalized();
      ex = ey.cross(ez).normalized();
      d_time = ros::Time::now();
      Eigen::Matrix3f m;
      m << ex, ey, ez;
      odom_transform = (m * odom_transform.rotation().inverse() * odom_transform);
      e_time = ros::Time::now();

      Eigen::Vector3f z = 0.032 * odom_transform.rotation() * Eigen::Vector3f::UnitZ();
      odom_transform(0, 3) += z[0];
      odom_transform(1, 3) += z[1];
      odom_transform(2, 3) += z[2];

      odom_transform = leg_transform * odom_transform;

      tf::Transform output_transform;
      tf::transformEigenToTF(odom_transform, output_transform);
      br.sendTransform(tf::StampedTransform(output_transform, stamps[0], "BODY", "odom_with_imu"));
      f_time = ros::Time::now();
    } catch (tf::TransformException ex){
      std::cout << "error" << std::endl;
    }
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
  if (msg->l_r == 0) {
    leg_frame = "rleg_end_coords";
  } else {
    leg_frame = "lleg_end_coords";
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_odom_publisher");
  OdomPublisher odom_publisher;
  return 0;
}
