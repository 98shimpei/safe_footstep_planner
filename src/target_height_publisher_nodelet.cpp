#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <jsk_recognition_utils/geo_util.h>
#include <safe_footstep_planner/OnlineFootStep.h>
#include <safe_footstep_planner/SteppableRegion.h>
#include <safe_footstep_planner/PolygonArray.h>
//#include <safe_footstep_planner/safe_footstep_util.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include "polypartition.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/core/eigen.hpp>
#include <jsk_recognition_msgs/HeightmapConfig.h>
#include <visualization_msgs/Marker.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>


namespace safe_footstep_planner
{
void matrixTFToEigen_ (const tf::Matrix3x3 &t, Eigen::Matrix3f &e)
{
  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
      e(i,j) = t[i][j];
}

void vectorTFToEigen_ (const tf::Vector3& t, Eigen::Vector3f& k)
{
  k(0) = t[0];
  k(1) = t[1];
  k(2) = t[2];
}

void calcFootRotFromNormal_ (Eigen::Matrix3f& foot_rot, const Eigen::Matrix3f& orig_rot, const Eigen::Vector3f& n)
{
  Eigen::Vector3f en = n / n.norm();
  Eigen::Vector3f ex = Eigen::Vector3f::UnitX();
  Eigen::Vector3f xv1(orig_rot * ex);
  xv1 = xv1 - xv1.dot(en) * en;
  xv1.normalize();
  Eigen::Vector3f yv1(en.cross(xv1));
  foot_rot(0,0) = xv1(0); foot_rot(1,0) = xv1(1); foot_rot(2,0) = xv1(2);
  foot_rot(0,1) = yv1(0); foot_rot(1,1) = yv1(1); foot_rot(2,1) = yv1(2);
  foot_rot(0,2) = en(0);  foot_rot(1,2) = en(1);  foot_rot(2,2) = en(2);
}

class TargetHeightPublisherNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber target_sub_;
  ros::Subscriber height_image_sub_;
  ros::Subscriber pose_image_sub_;
  ros::Subscriber yaw_image_sub_;
  ros::Publisher height_publisher_;
  ros::Publisher landing_pose_publisher_;
  tf::TransformListener listener_;
  void targetCallback(const safe_footstep_planner::OnlineFootStep::ConstPtr& msg);
  void heightImageCallback(const sensor_msgs::ImageConstPtr& msg);
  void poseImageCallback(const sensor_msgs::ImageConstPtr& msg);
  void yawImageCallback(const sensor_msgs::ImageConstPtr& msg);

  cv::Mat accumulated_height_image;
  cv::Mat accumulated_pose_image;
  cv::Mat accumulated_yaw_image;
  int accumulate_length;
  int accumulate_center_x;
  int accumulate_center_y;
  std::string fixed_frame;
  Eigen::Affine3f center_transform;

};

void TargetHeightPublisherNodelet::onInit()
{
  NODELET_INFO("TargetHeightPublisherNodelet Init");
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();
  height_publisher_ = nh_.advertise<safe_footstep_planner::OnlineFootStep>("landing_height", 1);
  landing_pose_publisher_ = nh_.advertise<visualization_msgs::Marker>("landing_pose_marker", 1);
  target_sub_ = nh_.subscribe("landing_target", 1, &TargetHeightPublisherNodelet::targetCallback, this);
  height_image_sub_ = nh_.subscribe("height_image_output", 1, &TargetHeightPublisherNodelet::heightImageCallback, this);
  pose_image_sub_ = nh_.subscribe("pose_image_output", 1, &TargetHeightPublisherNodelet::poseImageCallback, this);
  yaw_image_sub_ = nh_.subscribe("yaw_image_output", 1, &TargetHeightPublisherNodelet::yawImageCallback, this);

  pnh_.getParam("accumulate_length", accumulate_length);
  pnh_.getParam("accumulate_center_x", accumulate_center_x);
  pnh_.getParam("accumulate_center_y", accumulate_center_y);
  pnh_.getParam("fixed_frame", fixed_frame);
  accumulated_height_image = cv::Mat::zeros(accumulate_length, accumulate_length, CV_32FC1);
  accumulated_pose_image = cv::Mat(accumulate_length, accumulate_length, CV_32FC3, cv::Scalar(0, 0, 1));
  accumulated_yaw_image = cv::Mat::zeros(accumulate_length, accumulate_length, CV_32FC1);
}

void TargetHeightPublisherNodelet::heightImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  accumulated_height_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
}

void TargetHeightPublisherNodelet::poseImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  accumulated_pose_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC3)->image;
}

void TargetHeightPublisherNodelet::yawImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  accumulated_yaw_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
  tf::StampedTransform transform;
  listener_.waitForTransform(fixed_frame, msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
  listener_.lookupTransform(fixed_frame, msg->header.frame_id, msg->header.stamp, transform);
  tf::transformTFToEigen(transform, center_transform);
}

void TargetHeightPublisherNodelet::targetCallback(const safe_footstep_planner::OnlineFootStep::ConstPtr& msg)
{
  if (msg->l_r >= 2) {
    return;
  }
  double px = msg->x;
  double py = msg->y;
  double pz = msg->z;
  std::string target_frame;
  if (msg->l_r%2 == 1) {
    target_frame = "/lleg_end_coords";
  }
  else {
    target_frame = "/rleg_end_coords";
  }

  tf::StampedTransform transform;
  listener_.waitForTransform(fixed_frame, target_frame, msg->header.stamp, ros::Duration(3.0));
  listener_.lookupTransform(fixed_frame, target_frame, msg->header.stamp, transform);

  Eigen::Vector3f cur_foot_pos, ez(Eigen::Vector3f::UnitZ());
  vectorTFToEigen_(transform.getOrigin(), cur_foot_pos);
  Eigen::Matrix3f tmp_cur_foot_rot, cur_foot_rot;
  matrixTFToEigen_(transform.getBasis(), tmp_cur_foot_rot);
  calcFootRotFromNormal_(cur_foot_rot, tmp_cur_foot_rot, ez);
  Eigen::Vector3f next_foot_pos;
  next_foot_pos = cur_foot_pos + cur_foot_rot * Eigen::Vector3f(px, py, pz);

  Eigen::Vector3f cur_tmp = (center_transform * Eigen::Translation<float, 3>(-accumulate_center_x*0.01, -accumulate_center_y*0.01, 0)).inverse() * cur_foot_pos;
  Eigen::Vector3f next_tmp = (center_transform * Eigen::Translation<float, 3>(-accumulate_center_x*0.01, -accumulate_center_y*0.01, 0)).inverse() * next_foot_pos;
  if (0 <= (int)(cur_tmp[1]*100) && (int)(cur_tmp[1]*100) < accumulated_height_image.rows
   && 0 <= (int)(cur_tmp[0]*100) && (int)(cur_tmp[0]*100) < accumulated_height_image.cols
   && 0 <= (int)(next_tmp[1]*100) && (int)(next_tmp[1]*100) < accumulated_height_image.rows
   && 0 <= (int)(next_tmp[0]*100) && (int)(next_tmp[0]*100) < accumulated_height_image.cols) {
    cur_foot_pos[2] = accumulated_height_image.at<float>((int)(cur_tmp[1]*100), (int)(cur_tmp[0]*100));
    next_foot_pos[2] = accumulated_height_image.at<float>((int)(next_tmp[1]*100), (int)(next_tmp[0]*100));
  } else {
    return;
  }

  safe_footstep_planner::OnlineFootStep ps;
  ps.header = msg->header;
  ps.header.frame_id = target_frame.substr(1, target_frame.length() - 1);
  ps.l_r = msg->l_r;
  Eigen::Vector3f tmp_pose = Eigen::Vector3f(
    accumulated_pose_image.at<cv::Vec3f>((int)(next_tmp[1]*100), (int)(next_tmp[0]*100))[0],
    accumulated_pose_image.at<cv::Vec3f>((int)(next_tmp[1]*100), (int)(next_tmp[0]*100))[1],
    accumulated_pose_image.at<cv::Vec3f>((int)(next_tmp[1]*100), (int)(next_tmp[0]*100))[2]);
  //tmp_pose = cur_foot_rot.transpose() * Eigen::AngleAxisf(accumulated_yaw_image.at<float>((int)(next_tmp[1]*100), (int)(next_tmp[0]*100)), Eigen::Vector3f::UnitZ()) * tmp_pose;
  tmp_pose = Eigen::AngleAxisf(accumulated_yaw_image.at<float>((int)(next_tmp[1]*100), (int)(next_tmp[0]*100)), Eigen::Vector3f::UnitZ()) * tmp_pose;
  tmp_pose = cur_foot_rot.transpose() * tmp_pose;
  if (0.5 < tmp_pose.norm() && tmp_pose.norm() < 1.5) {
    ps.nx =  tmp_pose[0];
    ps.ny =  tmp_pose[1];
    ps.nz =  tmp_pose[2];
  } else {
    return;
  }

  Eigen::Vector3f tmp_pos;
  tmp_pos = cur_foot_rot.transpose() * (next_foot_pos - cur_foot_pos);
  tmp_pos[2] = std::abs(tmp_pos[2]) < 0.4 ? tmp_pos[2] : 0;
  ps.x = tmp_pos(0);
  ps.y = tmp_pos(1);
  ps.z = tmp_pos(2);

  height_publisher_.publish(ps);

  Eigen::Vector3f start_pos;
  start_pos = tmp_cur_foot_rot.transpose() * cur_foot_rot * Eigen::Vector3f(ps.x, ps.y, ps.z);
  Eigen::Vector3f end_pos;
  end_pos = tmp_cur_foot_rot.transpose() * cur_foot_rot * Eigen::Vector3f(ps.x+0.3*ps.nx, ps.y+0.3*ps.ny, ps.z+0.3*ps.nz);

  // publish pose msg for visualize
  visualization_msgs::Marker pose_msg;
  pose_msg.header = ps.header;
  pose_msg.ns = "landing_pose";
  pose_msg.id = 0;
  pose_msg.lifetime = ros::Duration();
  pose_msg.type = visualization_msgs::Marker::ARROW;
  pose_msg.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point start;
  start.x = start_pos(0);
  start.y = start_pos(1);
  start.z = start_pos(2);
  geometry_msgs::Point end;
  end.x = end_pos(0);
  end.y = end_pos(1);
  end.z = end_pos(2);
  pose_msg.points.push_back(start);
  pose_msg.points.push_back(end);
  pose_msg.color.r = 0.0;
  pose_msg.color.g = 0.8;
  pose_msg.color.b = 1.0;
  pose_msg.color.a = 1.0;
  pose_msg.scale.x = 0.03;
  pose_msg.scale.y = 0.05;
  pose_msg.scale.z = 0.07;
  pose_msg.pose.orientation.w = 1.0;

  landing_pose_publisher_.publish(pose_msg);
}
}
PLUGINLIB_EXPORT_CLASS(safe_footstep_planner::TargetHeightPublisherNodelet, nodelet::Nodelet)
