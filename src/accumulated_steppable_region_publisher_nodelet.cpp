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
#include <safe_footstep_planner/safe_footstep_util.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include "polypartition.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <jsk_recognition_msgs/HeightmapConfig.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>


namespace safe_footstep_planner
{
class SteppableRegionPublisherNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  safe_footstep_planner::PolygonArray combined_meshes_;
  ros::Subscriber target_sub_;
  ros::Subscriber heightmap_sub_;
  ros::Subscriber heightmap_config_sub_;
  ros::Publisher region_publisher_;
  ros::Publisher visualized_region_publisher_;
  ros::Publisher combined_mesh_publisher_;
  ros::Publisher steppable_image_publisher_;
  ros::Publisher height_image_publisher_;
  ros::Publisher pose_image_publisher_;
  ros::Publisher yaw_image_publisher_;
  ros::Publisher polygon_publisher_;
  //ros::Publisher height_publisher_;
  //ros::Publisher landing_pose_publisher_;
  tf::TransformListener listener_;
  void polygonarrayCallback(const jsk_recognition_msgs::PolygonArray::ConstPtr& msg);
  void targetCallback(const safe_footstep_planner::OnlineFootStep::ConstPtr& msg);
  void heightmapCallback(const sensor_msgs::ImageConstPtr& msg);
  void heightmapConfigCallback(const jsk_recognition_msgs::HeightmapConfigConstPtr& msg);
  cv::Mat median_image_;
  cv::Mat accumulated_steppable_image;
  cv::Mat accumulated_height_image;
  cv::Mat accumulated_pose_image;
  cv::Mat accumulated_yaw_image;
  cv::Mat trimmed_steppable_image;
  float cur_foot_pos_z;
  float grid_length;
  int steppable_range;
  float steppable_terrain_angle;
  float steppable_slope_angle;
  float obstacle_range;
  float obstacle_height;
  float height_limit;
  int erode_range;
  int accumulate_length;
  int accumulate_center_x;
  int accumulate_center_y;
  int trimmed_length;
  int trimmed_center_x;
  int trimmed_center_y;
  std::string fixed_frame;
  Eigen::Affine3f prev_transform;
  int heightmap_minx; //cm
  int heightmap_miny;
  int heightmap_maxx;
  int heightmap_maxy;
  bool heightmap_config_flag;
  std::string camera_frame;
  float fov_h;
  float fov_v;
  Eigen::Affine3f center_transform;
  Eigen::Affine3f camera_transform;
};

void SteppableRegionPublisherNodelet::onInit()
{
  NODELET_INFO("SteppableRegionPublisherNodelet Init");
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();
  region_publisher_ = nh_.advertise<safe_footstep_planner::SteppableRegion>("steppable_region", 1);
  visualized_region_publisher_ = nh_.advertise<jsk_recognition_msgs::PolygonArray>("visualized_steppable_region", 1);
  combined_mesh_publisher_ = nh_.advertise<safe_footstep_planner::PolygonArray>("combined_meshed_polygons", 1);
  steppable_image_publisher_ = nh_.advertise<sensor_msgs::Image> ("steppable_image_output", 1);
  height_image_publisher_ = nh_.advertise<sensor_msgs::Image> ("height_image_output", 1);
  pose_image_publisher_ = nh_.advertise<sensor_msgs::Image> ("pose_image_output", 1);
  yaw_image_publisher_ = nh_.advertise<sensor_msgs::Image> ("yaw_image_output", 1);
  polygon_publisher_ = nh_.advertise<jsk_recognition_msgs::PolygonArray> ("output_polygon", 1);
  //height_publisher_ = nh_.advertise<safe_footstep_planner::OnlineFootStep>("landing_height", 1);
  //landing_pose_publisher_ = nh_.advertise<visualization_msgs::Marker>("landing_pose_marker", 1);
  target_sub_ = nh_.subscribe("landing_target", 1, &SteppableRegionPublisherNodelet::targetCallback, this);
  heightmap_sub_ = nh_.subscribe("rt_current_heightmap/output", 1, &SteppableRegionPublisherNodelet::heightmapCallback, this);
  heightmap_config_sub_ = nh_.subscribe("rt_current_heightmap/output/config", 1, &SteppableRegionPublisherNodelet::heightmapConfigCallback, this);
  median_image_ = cv::Mat::zeros(500, 300, CV_32FC1);
  cur_foot_pos_z = 0;
  pnh_.getParam("grid_length", grid_length);
  pnh_.getParam("steppable_range", steppable_range);
  pnh_.getParam("steppable_terrain_angle", steppable_terrain_angle);
  pnh_.getParam("steppable_slope_angle", steppable_slope_angle);
  pnh_.getParam("obstacle_range", obstacle_range);
  pnh_.getParam("obstacle_height", obstacle_height);
  pnh_.getParam("height_limit", height_limit);
  pnh_.getParam("erode_range", erode_range);
  pnh_.getParam("accumulate_length", accumulate_length);
  pnh_.getParam("accumulate_center_x", accumulate_center_x);
  pnh_.getParam("accumulate_center_y", accumulate_center_y);
  pnh_.getParam("trimmed_length", trimmed_length);
  pnh_.getParam("trimmed_center_x", trimmed_center_x);
  pnh_.getParam("trimmed_center_y", trimmed_center_y);
  accumulated_steppable_image = cv::Mat::ones(accumulate_length, accumulate_length, CV_8UC1)*255;
  accumulated_height_image = cv::Mat::zeros(accumulate_length, accumulate_length, CV_32FC1);
  accumulated_pose_image = cv::Mat(accumulate_length, accumulate_length, CV_32FC3, cv::Scalar(0, 0, 1));
  accumulated_yaw_image = cv::Mat::zeros(accumulate_length, accumulate_length, CV_32FC1);
  trimmed_steppable_image = cv::Mat::ones(trimmed_length, trimmed_length, CV_8UC1)*255;
  pnh_.getParam("fixed_frame", fixed_frame);
  pnh_.getParam("camera_frame", camera_frame);
  pnh_.getParam("fov_h", fov_h);
  pnh_.getParam("fov_v", fov_v);
  prev_transform = Eigen::Affine3f();
  heightmap_config_flag = false;
}

void SteppableRegionPublisherNodelet::heightmapConfigCallback(const jsk_recognition_msgs::HeightmapConfigConstPtr& msg) {
  heightmap_config_flag = true;
  heightmap_minx = msg->min_x*100;
  heightmap_miny = msg->min_y*100;
  heightmap_maxx = msg->max_x*100;
  heightmap_maxy = msg->max_y*100;
}

void SteppableRegionPublisherNodelet::heightmapCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ros::Time begin_time = ros::Time::now();
  if (!heightmap_config_flag) return;
  try{
    tf::StampedTransform transform;
    listener_.waitForTransform(fixed_frame, msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
    listener_.lookupTransform(fixed_frame, msg->header.frame_id, msg->header.stamp, transform);
    tf::transformTFToEigen(transform, center_transform);
    if (prev_transform.translation().norm() <= 1e-10) {
      prev_transform = center_transform;
      return;
    }

    listener_.waitForTransform(msg->header.frame_id, camera_frame, msg->header.stamp, ros::Duration(1.0));
    listener_.lookupTransform(msg->header.frame_id, camera_frame, msg->header.stamp, transform);
    tf::transformTFToEigen(transform, camera_transform);

  } catch (tf::TransformException ex) {
    std::cout << "error" << std::endl;
    ROS_ERROR("%s", ex.what());
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat input_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC2)->image;  
  cv::medianBlur(input_image, median_image_, 5);
  cv::Mat current_height_image = cv::Mat::zeros(median_image_.rows, median_image_.cols, CV_32FC1);
  cv::medianBlur(median_image_, current_height_image, 5);

  cv::Mat binarized_image = cv::Mat::zeros(median_image_.rows, median_image_.cols, CV_8UC1);
  cv::Mat current_pose_image = cv::Mat::zeros(median_image_.rows, median_image_.cols, CV_32FC3);
  float steppable_terrain_edge_height = steppable_range*grid_length*0.01*std::tan(steppable_terrain_angle);
  float steppable_terrain_corner_height = steppable_terrain_edge_height*std::sqrt(2);//0.33
  float steppable_slope_edge_height = steppable_range*grid_length*0.01*std::tan(steppable_slope_angle);
  float steppable_slope_corner_height = steppable_slope_edge_height*std::sqrt(2);//0.33
  int obstacle_edge_range = (int)(obstacle_range/grid_length);//[cm]/[cm] 18
  int obstacle_corner_range = (int)(obstacle_edge_range/std::sqrt(2));//[cm]/[cm]

  float front = 0;
  float right = 0;
  float left = 0;
  float back = 0;
  float center = 0;
  bool front_flag = 0;
  bool right_flag = 0;
  bool left_flag = 0;
  bool back_flag= 0;

  ros::Time a_time = ros::Time::now();

  for (int x = (int)(obstacle_edge_range); x < (median_image_.cols-(int)(obstacle_edge_range)); x++) {
    for (int y = (int)(obstacle_edge_range); y < (median_image_.rows-(int)(obstacle_edge_range)); y++) {
      //if (std::abs(median_image_.at<cv::Vec2f>(y, x)[0] - cur_foot_pos_z) > height_limit) {
      //  continue;
      //}
      front = (median_image_.at<cv::Vec2f>(y-steppable_range, x-steppable_range)[0] + median_image_.at<cv::Vec2f>(y-steppable_range, x+steppable_range)[0]) / 2.0;
      front_flag = true;
      if (front + steppable_terrain_edge_height < median_image_.at<cv::Vec2f>(y-steppable_range, x)[0]) {
        front = median_image_.at<cv::Vec2f>(y-steppable_range, x)[0];
        front_flag = false;
      }
      right = (median_image_.at<cv::Vec2f>(y-steppable_range, x+steppable_range)[0] + median_image_.at<cv::Vec2f>(y+steppable_range, x+steppable_range)[0]) / 2.0;
      right_flag = true;
      if (right + steppable_terrain_edge_height < median_image_.at<cv::Vec2f>(y, x+steppable_range)[0]) {
        right = median_image_.at<cv::Vec2f>(y, x+steppable_range)[0];
        right_flag = false;
      }
      left = (median_image_.at<cv::Vec2f>(y-steppable_range, x-steppable_range)[0] + median_image_.at<cv::Vec2f>(y+steppable_range, x-steppable_range)[0]) / 2.0;
      left_flag = true;
      if (left + steppable_terrain_edge_height < median_image_.at<cv::Vec2f>(y, x-steppable_range)[0]) {
        left = median_image_.at<cv::Vec2f>(y, x-steppable_range)[0];
        left_flag = false;
      }
      back = (median_image_.at<cv::Vec2f>(y+steppable_range, x-steppable_range)[0] + median_image_.at<cv::Vec2f>(y+steppable_range, x+steppable_range)[0]) / 2.0;
      back_flag = true;
      if (back + steppable_terrain_edge_height < median_image_.at<cv::Vec2f>(y+steppable_range, x)[0]) {
        back = median_image_.at<cv::Vec2f>(y+steppable_range, x)[0];
        back_flag = false;
      }

      ////凸性
      if (std::abs(front + back - right - left) > steppable_terrain_edge_height*2) {
        continue;
      }
      center = (front + back + right + left) / 4.0;
      if (center + steppable_terrain_edge_height < median_image_.at<cv::Vec2f>(y, x)[0]) {
        continue;
      }

      //ひねり
      if (front_flag && std::abs(median_image_.at<cv::Vec2f>(y-steppable_range, x-steppable_range)[0] - median_image_.at<cv::Vec2f>(y-steppable_range, x+steppable_range)[0] + right - left) > steppable_terrain_edge_height) {
        continue;
      }
      if (right_flag && std::abs(median_image_.at<cv::Vec2f>(y-steppable_range, x+steppable_range)[0] - median_image_.at<cv::Vec2f>(y+steppable_range, x+steppable_range)[0] + back - front) > steppable_terrain_edge_height) {
        continue;
      }
      if (left_flag && std::abs(median_image_.at<cv::Vec2f>(y-steppable_range, x-steppable_range)[0] - median_image_.at<cv::Vec2f>(y+steppable_range, x-steppable_range)[0] + back - front) > steppable_terrain_edge_height) {
        continue;
      }
      if (back_flag && std::abs(median_image_.at<cv::Vec2f>(y+steppable_range, x-steppable_range)[0] - median_image_.at<cv::Vec2f>(y+steppable_range, x+steppable_range)[0] + right - left) > steppable_terrain_edge_height) {
        continue;
      }

      ////傾き
      if (std::abs(front - back) > steppable_slope_edge_height*2) {
        continue;
      }
      if (std::abs(right - left) > steppable_slope_edge_height*2) {
        continue;
      }

      //image.at<cv::Vec3b>(y, x)[0] = 100;
      //image.at<cv::Vec3b>(y, x)[1] = 100;
      //image.at<cv::Vec3b>(y, x)[2] = 100;

      if (
        median_image_.at<cv::Vec2f>(y+obstacle_edge_range, x)[0] - center > obstacle_height ||
        median_image_.at<cv::Vec2f>(y, x+obstacle_edge_range)[0] - center > obstacle_height ||
        median_image_.at<cv::Vec2f>(y-obstacle_edge_range, x)[0] - center > obstacle_height ||
        median_image_.at<cv::Vec2f>(y, x-obstacle_edge_range)[0] - center > obstacle_height ||
        median_image_.at<cv::Vec2f>(y+obstacle_corner_range, x+obstacle_corner_range)[0] - center > obstacle_height ||
        median_image_.at<cv::Vec2f>(y+obstacle_corner_range, x-obstacle_corner_range)[0] - center > obstacle_height ||
        median_image_.at<cv::Vec2f>(y-obstacle_corner_range, x+obstacle_corner_range)[0] - center > obstacle_height ||
        median_image_.at<cv::Vec2f>(y-obstacle_corner_range, x-obstacle_corner_range)[0] - center > obstacle_height) {
        continue;
      }
      binarized_image.at<uchar>(y, x) = 255;
      //image.at<cv::Vec3b>(y, x)[0] = 200;
      //image.at<cv::Vec3b>(y, x)[1] = 200;
      //image.at<cv::Vec3b>(y, x)[2] = 200;
    }
  }

  ros::Time b_time = ros::Time::now();

  std::vector<int> x_list = {-steppable_range, (int)(-steppable_range*1/2), 0, (int)(steppable_range*1/2), steppable_range};
  std::vector<int> y_list = {-steppable_range, (int)(-steppable_range*1/2), 0, (int)(steppable_range*1/2), steppable_range};

  for (int x = (int)(obstacle_edge_range); x < (median_image_.cols-(int)(obstacle_edge_range)); x+=2) {
    for (int y = (int)(obstacle_edge_range); y < (median_image_.rows-(int)(obstacle_edge_range)); y+=2) {
      //std::vector<int> xv;
      //std::vector<int> yv;
      //for (int p = 0; p < x_list.size(); p++) {
      //  for (int q = 0; q < y_list.size(); q++) {
      //    if (median_image_.at<cv::Vec2f>(y+y_list[q], x+x_list[p])[0] > -1e+10) {
      //      xv.push_back(x+x_list[p]);
      //      yv.push_back(y+y_list[q]);
      //    }
      //  }
      //}
      //
      //if (xv.size() < 3) {
      //  current_pose_image.at<cv::Vec4f>(y, x)[1] = 0;
      //  current_pose_image.at<cv::Vec4f>(y, x)[2] = 0;
      //  current_pose_image.at<cv::Vec4f>(y, x)[3] = 1;
      //  continue;
      //}

      //cv::Mat data_pts = cv::Mat(xv.size(), 3, CV_32F);
      //for (int i = 0; i < xv.size(); i++) {
      //  data_pts.at<float>(i, 0) = xv[i]*0.01;
      //  data_pts.at<float>(i, 1) = yv[i]*0.01;
      //  data_pts.at<float>(i, 2) = median_image_.at<cv::Vec2f>(yv[i], xv[i])[0];
      //}


      cv::Mat data_pts = cv::Mat(x_list.size() * y_list.size(), 3, CV_32F);
      bool flag = false;
      for (int p = 0; p < x_list.size(); p++) {
        for (int q = 0; q < y_list.size(); q++) {
          if (median_image_.at<cv::Vec2f>(y+y_list[q], x+x_list[p])[0] < -1e+10) {
            flag = true;
            break;
          }
          data_pts.at<float>(p*y_list.size()+q, 0) = (x+x_list[p])*0.01;
          data_pts.at<float>(p*y_list.size()+q, 1) = (y+y_list[q])*0.01;
          data_pts.at<float>(p*y_list.size()+q, 2) = median_image_.at<cv::Vec2f>(y+y_list[q], x+x_list[p])[0];
        }
      }
      Eigen::Vector3f tmp;
      if (flag) {
        tmp[0] = -1e+20;
        tmp[1] = -1e+20;
        tmp[2] = -1e+20;
      } else {
        cv::PCA pca_analysis(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);
        tmp[0] = pca_analysis.eigenvectors.at<float>(2, 0);
        tmp[1] = pca_analysis.eigenvectors.at<float>(2, 1);
        tmp[2] = pca_analysis.eigenvectors.at<float>(2, 2);
        if (tmp[2] < 0) tmp = -tmp;
        if (tmp[2] < std::cos(0.76)) {
          tmp[0] = -1e+20;
          tmp[1] = -1e+20;
          tmp[2] = -1e+20;
        }
      }
      current_pose_image.at<cv::Vec3f>(y, x)[0]     = tmp[0];
      current_pose_image.at<cv::Vec3f>(y, x)[1]     = tmp[1];
      current_pose_image.at<cv::Vec3f>(y, x)[2]     = tmp[2];
      current_pose_image.at<cv::Vec3f>(y, x+1)[0]   = tmp[0];
      current_pose_image.at<cv::Vec3f>(y, x+1)[1]   = tmp[1];
      current_pose_image.at<cv::Vec3f>(y, x+1)[2]   = tmp[2];
      current_pose_image.at<cv::Vec3f>(y+1, x)[0]   = tmp[0];
      current_pose_image.at<cv::Vec3f>(y+1, x)[1]   = tmp[1];
      current_pose_image.at<cv::Vec3f>(y+1, x)[2]   = tmp[2];
      current_pose_image.at<cv::Vec3f>(y+1, x+1)[0] = tmp[0];
      current_pose_image.at<cv::Vec3f>(y+1, x+1)[1] = tmp[1];
      current_pose_image.at<cv::Vec3f>(y+1, x+1)[2] = tmp[2];
    }
  }

  ros::Time c_time = ros::Time::now();

  //蓄積

  Eigen::Affine3f tmp_trans = (center_transform * Eigen::Translation<float, 3>(-accumulate_center_x*0.01, -accumulate_center_y*0.01, 0)).inverse() * (prev_transform * Eigen::Translation<float, 3>(-accumulate_center_x*0.01, -accumulate_center_y*0.01, 0));
  Eigen::Matrix<float, 2, 3> m;
  m << tmp_trans(0,0), tmp_trans(0,1), tmp_trans(0,3)*100,
       tmp_trans(1,0), tmp_trans(1,1), tmp_trans(1,3)*100;
  cv::Mat tmp_trans_cv;
  cv::eigen2cv(m, tmp_trans_cv);
  cv::warpAffine(accumulated_steppable_image, accumulated_steppable_image, tmp_trans_cv, accumulated_steppable_image.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(255));
  cv::warpAffine(accumulated_height_image, accumulated_height_image, tmp_trans_cv, accumulated_height_image.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
  cv::warpAffine(accumulated_pose_image, accumulated_pose_image, tmp_trans_cv, accumulated_pose_image.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 1));
  cv::warpAffine(accumulated_yaw_image, accumulated_yaw_image, tmp_trans_cv, accumulated_yaw_image.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));

  Eigen::Vector3f ea = camera_transform.rotation() * Eigen::Vector3f(std::tan(deg2rad(fov_h/2.0)), std::tan(deg2rad(fov_v/2.0)), 1.0);
  Eigen::Vector3f eb = camera_transform.rotation() * Eigen::Vector3f(std::tan(deg2rad(fov_h/2.0)), -std::tan(deg2rad(fov_v/2.0)), 1.0);
  Eigen::Vector3f ec = camera_transform.rotation() * Eigen::Vector3f(-std::tan(deg2rad(fov_h/2.0)), -std::tan(deg2rad(fov_v/2.0)), 1.0);
  Eigen::Vector3f ed = camera_transform.rotation() * Eigen::Vector3f(-std::tan(deg2rad(fov_h/2.0)), std::tan(deg2rad(fov_v/2.0)), 1.0);
  ea = ea * std::abs(camera_transform(2, 3) / ea[2]) * 100 - Eigen::Vector3f(heightmap_minx, heightmap_miny, 0);
  eb = eb * std::abs(camera_transform(2, 3) / eb[2]) * 100 - Eigen::Vector3f(heightmap_minx, heightmap_miny, 0);
  ec = ec * std::abs(camera_transform(2, 3) / ec[2]) * 100 - Eigen::Vector3f(heightmap_minx, heightmap_miny, 0);
  ed = ed * std::abs(camera_transform(2, 3) / ed[2]) * 100 - Eigen::Vector3f(heightmap_minx, heightmap_miny, 0);

  Eigen::Vector3f tmp_vec = center_transform.rotation() * Eigen::Vector3f::UnitX();
  float tmp_yaw = std::atan2(tmp_vec[1], tmp_vec[0]);
  for (int x = std::max(0, (int)(ea[0])); x < std::min((int)(ec[0]), binarized_image.cols); x++) {
    for (int y = std::max(0, (int)(ea[1] + (eb[1]-ea[1])*(x-ea[0])/(eb[0]-ea[0]))); y < std::min((int)(ed[1] + (ec[1]-ed[1])*(x-ed[0])/(ec[0]-ed[0])), binarized_image.rows); y++) {
      accumulated_steppable_image.at<uchar>(y+heightmap_miny+accumulate_center_y, x+heightmap_minx+accumulate_center_x) = binarized_image.at<uchar>(y, x);
      if (current_height_image.at<cv::Vec2f>(y, x)[0] > -1e+10) {
        accumulated_height_image.at<float>(y+heightmap_miny+accumulate_center_y, x+heightmap_minx+accumulate_center_x) = current_height_image.at<cv::Vec2f>(y, x)[0];
      }
      if (current_pose_image.at<cv::Vec3f>(y, x)[0] > -1e+10) {
        accumulated_pose_image.at<cv::Vec3f>(y+heightmap_miny+accumulate_center_y, x+heightmap_minx+accumulate_center_x)[0] = current_pose_image.at<cv::Vec3f>(y, x)[0];
        accumulated_pose_image.at<cv::Vec3f>(y+heightmap_miny+accumulate_center_y, x+heightmap_minx+accumulate_center_x)[1] = current_pose_image.at<cv::Vec3f>(y, x)[1];
        accumulated_pose_image.at<cv::Vec3f>(y+heightmap_miny+accumulate_center_y, x+heightmap_minx+accumulate_center_x)[2] = current_pose_image.at<cv::Vec3f>(y, x)[2];
      }
      accumulated_yaw_image.at<float>(y+heightmap_miny+accumulate_center_y, x+heightmap_minx+accumulate_center_x) = tmp_yaw; //yaw
    }
  }

  prev_transform = center_transform;

  ros::Time d_time = ros::Time::now();

  //拡大縮小、トリミング
  cv::Mat eroded_image;
  trimmed_steppable_image = accumulated_steppable_image(cv::Rect(accumulate_center_x - trimmed_center_x, accumulate_center_y - trimmed_center_y, trimmed_length, trimmed_length)).clone();
  
  cv::morphologyEx(accumulated_steppable_image, eroded_image, CV_MOP_CLOSE, cv::noArray(), cv::Point(-1, -1), 2);
  cv::morphologyEx(eroded_image, eroded_image, CV_MOP_OPEN,  cv::noArray(), cv::Point(-1, -1), 1);
  cv::erode(eroded_image, eroded_image, cv::noArray(), cv::Point(-1, -1), erode_range);//3
  cv::morphologyEx(eroded_image, eroded_image, CV_MOP_OPEN, cv::noArray(), cv::Point(-1, -1), 1);

  cv::morphologyEx(trimmed_steppable_image, trimmed_steppable_image, CV_MOP_CLOSE, cv::noArray(), cv::Point(-1, -1), 2);
  cv::morphologyEx(trimmed_steppable_image, trimmed_steppable_image, CV_MOP_OPEN,  cv::noArray(), cv::Point(-1, -1), 1);
  cv::erode(trimmed_steppable_image, trimmed_steppable_image, cv::noArray(), cv::Point(-1, -1), erode_range);//3
  cv::morphologyEx(trimmed_steppable_image, trimmed_steppable_image, CV_MOP_OPEN, cv::noArray(), cv::Point(-1, -1), 1);


  //輪郭抽出
  std::vector<std::vector<cv::Point>> approx_vector;
  std::list<TPPLPoly> polys, result;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(trimmed_steppable_image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

  int size_threshold = 5;
  for (int j = 0; j < contours.size(); j++) {
    if (hierarchy[j][3] == -1) { //外側
      if (cv::contourArea(contours[j]) > size_threshold) {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[j], approx, 0.9, true);//1.5
        if (approx.size() >= 3) {
          approx_vector.push_back(approx);
          TPPLPoly poly;
          poly.Init(approx.size());
          for (int k = 0; k < approx.size(); k++) {
            poly[k].x = approx[k].x;
            poly[k].y = -approx[k].y; //TPPLPartitionのために右回り・左回りを反転させる
          }
          polys.push_back(poly);
        }
      }
    } else { //穴
      if (cv::contourArea(contours[j]) > size_threshold) {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[j], approx, 1.0, true);
        if (approx.size() >= 3) {
          approx_vector.push_back(approx);
          TPPLPoly poly;
          poly.Init(approx.size());
          for (int k = 0; k < approx.size(); k++) {
            poly[k].x = approx[k].x;
            poly[k].y = -approx[k].y;
          }
          poly.SetHole(true);
          polys.push_back(poly);
        }
      }
    }
  }

  TPPLPartition pp;
  pp.Triangulate_EC(&polys, &result);

  //cv::drawContours(eroded_image, approx_vector, -1, cv::Scalar(100));

  ros::Time e_time = ros::Time::now();

  jsk_recognition_msgs::PolygonArray polygon_msg;
  polygon_msg.header = std_msgs::Header();
  polygon_msg.header.frame_id = fixed_frame;
  polygon_msg.header.stamp = msg->header.stamp;
  safe_footstep_planner::PolygonArray meshed_polygons_msg;

  std::list<TPPLPoly>::iterator iter;
  for (iter = result.begin(); iter != result.end(); iter++) {
    geometry_msgs::PolygonStamped ps;
    for (int j = iter->GetNumPoints() - 1; j >= 0; j--) {//逆順
      //反転していたy座標をもとに戻す
      Eigen::Vector3f tmp = Eigen::Vector3f((iter->GetPoint(j).x - trimmed_center_x)*0.01, (-iter->GetPoint(j).y-trimmed_center_y) * 0.01, (accumulated_height_image.at<float>(-iter->GetPoint(j).y + accumulate_center_y - trimmed_center_y, iter->GetPoint(j).x + accumulate_center_x - trimmed_center_x) < -1e+10) ? 0 : accumulated_height_image.at<float>(-iter->GetPoint(j).y + accumulate_center_y - trimmed_center_y, iter->GetPoint(j).x + accumulate_center_x - trimmed_center_x));
      tmp = center_transform * tmp;
      geometry_msgs::Point32 p;
      p.x = tmp[0];
      p.y = tmp[1];
      p.z = tmp[2];
      ps.polygon.points.push_back(p);
    }
    ps.header = std_msgs::Header();
    ps.header.frame_id = fixed_frame;
    ps.header.stamp = ros::Time(0);
    polygon_msg.polygons.push_back(ps);
    meshed_polygons_msg.polygons.push_back(ps.polygon);
  }

  polygon_publisher_.publish(polygon_msg);

  cv::line(eroded_image, cv::Point(ea[0]+accumulate_center_x+heightmap_minx, ea[1]+accumulate_center_y+heightmap_miny), cv::Point(eb[0]+accumulate_center_x+heightmap_minx, eb[1]+accumulate_center_y+heightmap_miny), cv::Scalar(180), 3);
  cv::line(eroded_image, cv::Point(eb[0]+accumulate_center_x+heightmap_minx, eb[1]+accumulate_center_y+heightmap_miny), cv::Point(ec[0]+accumulate_center_x+heightmap_minx, ec[1]+accumulate_center_y+heightmap_miny), cv::Scalar(180), 3);
  cv::line(eroded_image, cv::Point(ec[0]+accumulate_center_x+heightmap_minx, ec[1]+accumulate_center_y+heightmap_miny), cv::Point(ed[0]+accumulate_center_x+heightmap_minx, ed[1]+accumulate_center_y+heightmap_miny), cv::Scalar(180), 3);
  cv::line(eroded_image, cv::Point(ed[0]+accumulate_center_x+heightmap_minx, ed[1]+accumulate_center_y+heightmap_miny), cv::Point(ea[0]+accumulate_center_x+heightmap_minx, ea[1]+accumulate_center_y+heightmap_miny), cv::Scalar(180), 3);

  cv::flip(eroded_image, eroded_image, 0);
  //cv::flip(image, image, 0);
  //image_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());
  steppable_image_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", eroded_image).toImageMsg());

  cv::Mat output_height_image = cv::Mat::zeros(accumulated_height_image.cols, accumulated_height_image.rows, CV_8UC3);
  cv::Mat output_pose_image = cv::Mat::zeros(accumulated_height_image.cols, accumulated_height_image.rows, CV_8UC3);
  float th1 = -0.1, th2 = 0.1, th3 = 0.3, th4 = 0.5;
  for (int y = 0; y < accumulated_height_image.cols; y++) {
    for (int x = 0; x < accumulated_height_image.cols; x++) {
      float tmp = accumulated_height_image.at<float>(y, x);
      if (tmp < th1 || tmp > th4) {
        output_height_image.at<cv::Vec3b>(y, x)[0] = 0;
        output_height_image.at<cv::Vec3b>(y, x)[1] = 0;
        output_height_image.at<cv::Vec3b>(y, x)[2] = 0;
      } else if (tmp < th2) {
        output_height_image.at<cv::Vec3b>(y, x)[0] = (int)((tmp-th1)*250/0.5);
        output_height_image.at<cv::Vec3b>(y, x)[1] = (int)((th2-tmp)*250/0.5);
        output_height_image.at<cv::Vec3b>(y, x)[2] = 0;
      } else if (tmp < th3) {
        output_height_image.at<cv::Vec3b>(y, x)[0] = (int)((th3-tmp)*250/0.5);
        output_height_image.at<cv::Vec3b>(y, x)[1] = 0;
        output_height_image.at<cv::Vec3b>(y, x)[2] = (int)((tmp-th2)*250/0.5);
      } else {
        output_height_image.at<cv::Vec3b>(y, x)[0] = 0;
        output_height_image.at<cv::Vec3b>(y, x)[1] = (int)((tmp-th3)*250/0.5);
        output_height_image.at<cv::Vec3b>(y, x)[2] = (int)((th4-tmp)*250/0.5);
      }
      output_pose_image.at<cv::Vec3b>(y, x)[0] = std::min(255, std::max(0, (int)(accumulated_pose_image.at<cv::Vec3f>(y, x)[0]*700)+125));
      output_pose_image.at<cv::Vec3b>(y, x)[1] = std::min(255, std::max(0, (int)(accumulated_pose_image.at<cv::Vec3f>(y, x)[1]*700)+125));
      output_pose_image.at<cv::Vec3b>(y, x)[2] = 125;
    }
  }
  //height_image_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_height_image).toImageMsg());
  //pose_image_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_pose_image).toImageMsg());

  height_image_publisher_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_32FC1, accumulated_height_image).toImageMsg());
  pose_image_publisher_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_32FC3, accumulated_pose_image).toImageMsg());
  yaw_image_publisher_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_32FC1, accumulated_yaw_image).toImageMsg());

  ros::Time f_time = ros::Time::now();


  std::vector<std::vector<Eigen::Vector3f> > meshes;
  std::vector<std::vector<Eigen::Vector3f> > combined_meshes;
  std::vector<std::vector<size_t> > combined_indices;

  // convert to Eigen::Vector3f
  size_t mesh_num(meshed_polygons_msg.polygons.size());
  meshes.resize(mesh_num);
  // std::cerr << "mesh_num : " << mesh_num << std::endl;
  std::vector<bool> is_combined(mesh_num, false);
  for (size_t i = 0; i < mesh_num; i++) {
    size_t vs_num(meshed_polygons_msg.polygons[i].points.size()); // must be 3 (triangle)
    meshes[i].resize(vs_num);
    for (size_t j = 0; j < vs_num; j++) {
      safe_footstep_util::pointsToEigen(meshed_polygons_msg.polygons[i].points[j], meshes[i][j]);
    }
  }

  // debug
  // size_t mesh_num(4);
  // std::vector<bool> is_combined(mesh_num, false);
  // meshes.resize(mesh_num);
  // // meshes[0].push_back(Eigen::Vector3f(0, 0, 0));
  // // meshes[0].push_back(Eigen::Vector3f(500, 0, 0));
  // // meshes[0].push_back(Eigen::Vector3f(700, 500, 0));
  // // meshes[1].push_back(Eigen::Vector3f(400, 800, 0));
  // // meshes[1].push_back(Eigen::Vector3f(700, 500, 0));
  // // meshes[1].push_back(Eigen::Vector3f(1000, 700, 0));
  // // meshes[2].push_back(Eigen::Vector3f(700, 500, 0));
  // // meshes[2].push_back(Eigen::Vector3f(400, 800, 0));
  // // meshes[2].push_back(Eigen::Vector3f(0, 0, 0));
  // // meshes[3].push_back(Eigen::Vector3f(1000, 700, 0));
  // // meshes[3].push_back(Eigen::Vector3f(650, 850, 0));
  // // meshes[3].push_back(Eigen::Vector3f(400, 800, 0));
  // meshes[0].push_back(Eigen::Vector3f(-0.5, -1, 0));
  // meshes[0].push_back(Eigen::Vector3f(-0.5, 1, 0));
  // meshes[0].push_back(Eigen::Vector3f(0.5, 1, 0));
  // meshes[1].push_back(Eigen::Vector3f(-0.5, -1, 0));
  // meshes[1].push_back(Eigen::Vector3f(0.5, 1, 0));
  // meshes[1].push_back(Eigen::Vector3f(0.5, -1, 0));
  // meshes[2].push_back(Eigen::Vector3f(0.7, -1.3, 0));
  // meshes[2].push_back(Eigen::Vector3f(0.7, 1.3, 0));
  // meshes[2].push_back(Eigen::Vector3f(1.7, 1.3, 0));
  // meshes[3].push_back(Eigen::Vector3f(0.7, -1.3, 0));
  // meshes[3].push_back(Eigen::Vector3f(1.7, 1.3, 0));
  // meshes[3].push_back(Eigen::Vector3f(1.7, -1.3, 0));

  // combine meshes
  for (size_t i = 0; i < meshes.size(); i++) {
    std::vector<size_t> is_combined_indices;
    is_combined_indices.push_back(i);
    for (size_t j = i + 1; j < meshes.size(); j++) {
      std::vector<Eigen::Vector3f> inter_v;
      std::vector<Eigen::Vector3f> v1 = meshes[i], v2 = meshes[j];
      std::sort(v1.begin(), v1.end(), safe_footstep_util::compare_eigen3f);
      std::sort(v2.begin(), v2.end(), safe_footstep_util::compare_eigen3f);
      std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), inserter(inter_v, inter_v.end()), safe_footstep_util::compare_eigen3f);
      if (inter_v.size() == 2) { // adjacent mesh
        std::vector<Eigen::Vector3f> tmp_vs(v1), target_v, tmp_convex;
        std::set_difference(v2.begin(), v2.end(), v1.begin(), v1.end(), inserter(target_v, target_v.end()), safe_footstep_util::compare_eigen3f);
        std::copy(target_v.begin(), target_v.end(), std::back_inserter(tmp_vs));
        safe_footstep_util::calc_convex_hull(tmp_vs, tmp_convex);
        if (tmp_vs.size() == tmp_convex.size()) {
          meshes[i] = tmp_convex;
          meshes[j] = tmp_convex;
          is_combined[j] = true;
          is_combined_indices.push_back(j);
        }
      }
    }
    if (!is_combined[i]) {
      combined_meshes.push_back(meshes[i]);
      combined_indices.push_back(is_combined_indices);
    } else if (is_combined_indices.size() > 1) {
      for (size_t j = 0; j < combined_indices.size(); j++) {
        if (std::find(combined_indices[j].begin(), combined_indices[j].end(), i) != combined_indices[j].end()) {
          combined_meshes[j] = meshes[i];
          combined_indices[j] = is_combined_indices;
        }
      }
    }
    is_combined[i] = true;
  }

  // // add near mesh
  // tf::StampedTransform transform;
  // listener_.lookupTransform("/map", "/rleg_end_coords", ros::Time(0), transform); // map relative to rleg
  // Eigen::Vector3f rfoot_pos;
  // safe_footstep_util::vectorTFToEigen(transform.getOrigin(), rfoot_pos);
  // std::vector<Eigen::Vector3f> near_mesh(4);
  // near_mesh[0] = Eigen::Vector3f(rfoot_pos(0)-0.15, rfoot_pos(1)-0.25, 0);
  // near_mesh[1] = Eigen::Vector3f(rfoot_pos(0)+0.15, rfoot_pos(1)-0.25, 0);
  // near_mesh[2] = Eigen::Vector3f(rfoot_pos(0)+0.15, rfoot_pos(1)+0.45, 0);
  // near_mesh[3] = Eigen::Vector3f(rfoot_pos(0)-0.15, rfoot_pos(1)+0.45, 0);
  // combined_meshes.push_back(near_mesh);

  // convert to safe_footstep_planner::PolygonArray
  size_t combined_mesh_num(combined_meshes.size());
  // std::cerr << "combined_mesh_num : " << combined_mesh_num << std::endl;
  combined_meshes_.polygons.resize(combined_mesh_num);
  for (size_t i = 0; i < combined_mesh_num; i++) {
    size_t vs_num(combined_meshes[i].size());
    combined_meshes_.polygons[i].points.resize(vs_num);
    for (size_t j = 0; j < vs_num; j++) {
      safe_footstep_util::eigenToPoints(combined_meshes[i][j], combined_meshes_.polygons[i].points[j]);
    }
  }

  combined_mesh_publisher_.publish(combined_meshes_);

  ros::Time end_time = ros::Time::now();
  //std::cout << "all_time " << (end_time - begin_time).sec << "s " << (int)((end_time - begin_time).nsec / 1000000) << "ms" << std::endl;
  //std::cout << "begin_a  " << (a_time - begin_time).sec << "s " << (int)((a_time - begin_time).nsec / 1000000) << "ms" << std::endl;
  //std::cout << "a_b  " << (b_time - a_time).sec << "s " << (int)((b_time - a_time).nsec / 1000000) << "ms" << std::endl;
  //std::cout << "b_c  " << (c_time - b_time).sec << "s " << (int)((c_time - b_time).nsec / 1000000) << "ms" << std::endl;
  //std::cout << "c_d  " << (d_time - c_time).sec << "s " << (int)((d_time - c_time).nsec / 1000000) << "ms" << std::endl;
  //std::cout << "d_e  " << (e_time - d_time).sec << "s " << (int)((e_time - d_time).nsec / 1000000) << "ms" << std::endl;
  //std::cout << "e_f  " << (f_time - e_time).sec << "s " << (int)((f_time - e_time).nsec / 1000000) << "ms" << std::endl;
  //std::cout << "f_end  " << (end_time - f_time).sec << "s " << (int)((end_time - f_time).nsec / 1000000) << "ms" << std::endl;

}

void SteppableRegionPublisherNodelet::targetCallback(const safe_footstep_planner::OnlineFootStep::ConstPtr& msg)
{
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
  safe_footstep_util::vectorTFToEigen(transform.getOrigin(), cur_foot_pos);
  Eigen::Matrix3f tmp_cur_foot_rot, cur_foot_rot;
  safe_footstep_util::matrixTFToEigen(transform.getBasis(), tmp_cur_foot_rot);
  safe_footstep_util::calcFootRotFromNormal(cur_foot_rot, tmp_cur_foot_rot, ez);
  Eigen::Vector3f next_foot_pos;
  next_foot_pos = cur_foot_pos + cur_foot_rot * Eigen::Vector3f(px, py, pz);


  //if (x_x_diff != 0 || x_y_diff != 0) {
  //  double norm = std::sqrt(x_x_diff*x_x_diff + x_y_diff*x_y_diff);
  //  Eigen::Matrix2d tmpmat;
  //  tmpmat << x_x_diff, -x_y_diff,
  //            x_y_diff,  x_x_diff;
  //  Eigen::Vector2d tmpvec;
  //  tmpvec << cur_foot_pos[0] - median_image_.at<cv::Vec3f>(0,0)[0], cur_foot_pos[1] - median_image_.at<cv::Vec3f>(0,0)[1];
  //  Eigen::Vector2d tmp;
  //  tmp = tmpmat.colPivHouseholderQr().solve(tmpvec);
  //  cur_foot_pos[2] = median_image_.at<cv::Vec3f>((int)(tmp[1]), (int)(tmp[0]))[2];
  //  cur_foot_pos_z = cur_foot_pos[2];
  //  //std::cout << x_x_diff << " " << x_y_diff << "  " << cur_foot_pos[0] << " " << cur_foot_pos[1] << "  " << median_image_.at<cv::Vec3f>(0, 0)[0] << " " << median_image_.at<cv::Vec3f>(0, 0)[1] << " " << median_image_.at<cv::Vec3f>(0, 0)[2] << "  " << tmp[0] << " " << tmp[1] << "  " << cur_foot_pos[2] << std::endl;
  //}

  safe_footstep_planner::SteppableRegion sr;
  std_msgs::Header header;
  header.frame_id = target_frame.substr(1, target_frame.length() - 1);
  header.stamp = ros::Time(0);
  sr.header = header;
  sr.l_r = msg->l_r;

  // convert to polygon relative to leg_end_coords
  size_t convex_num(combined_meshes_.polygons.size());
  sr.polygons.resize(convex_num);
  for (size_t i = 0; i < convex_num; i++) {
    size_t vs_num(combined_meshes_.polygons[i].points.size());
    sr.polygons[i].polygon.points.resize(vs_num);
    for (size_t j = 0; j < vs_num; j++) {
      safe_footstep_util::transformPoint(combined_meshes_.polygons[i].points[j], cur_foot_rot, cur_foot_pos, sr.polygons[i].polygon.points[j]);
    }
  }

  region_publisher_.publish(sr);

  //Eigen::Vector3f cur_tmp = (center_transform * Eigen::Translation<float, 3>(-accumulate_center_x*0.01, -accumulate_center_y*0.01, 0)).inverse() * cur_foot_pos;
  //Eigen::Vector3f next_tmp = (center_transform * Eigen::Translation<float, 3>(-accumulate_center_x*0.01, -accumulate_center_y*0.01, 0)).inverse() * next_foot_pos;
  //if (0 <= (int)(cur_tmp[1]*100) && (int)(cur_tmp[1]*100) < accumulated_height_image.rows
  // && 0 <= (int)(cur_tmp[0]*100) && (int)(cur_tmp[0]*100) < accumulated_height_image.cols
  // && 0 <= (int)(next_tmp[1]*100) && (int)(next_tmp[1]*100) < accumulated_height_image.rows
  // && 0 <= (int)(next_tmp[0]*100) && (int)(next_tmp[0]*100) < accumulated_height_image.cols) {
  //  cur_foot_pos[2] = accumulated_height_image.at<float>((int)(cur_tmp[1]*100), (int)(cur_tmp[0]*100));
  //  next_foot_pos[2] = accumulated_height_image.at<float>((int)(next_tmp[1]*100), (int)(next_tmp[0]*100));
  //} else {
  //  return;
  //}

  //safe_footstep_planner::OnlineFootStep ps;
  //ps.header = msg->header;
  //ps.header.frame_id = target_frame.substr(1, target_frame.length() - 1);
  //ps.l_r = msg->l_r;
  //Eigen::Vector3f tmp_pose = Eigen::Vector3f(
  //  accumulated_pose_image.at<cv::Vec3f>((int)(next_tmp[1]*100), (int)(next_tmp[0]*100))[0],
  //  accumulated_pose_image.at<cv::Vec3f>((int)(next_tmp[1]*100), (int)(next_tmp[0]*100))[1],
  //  accumulated_pose_image.at<cv::Vec3f>((int)(next_tmp[1]*100), (int)(next_tmp[0]*100))[2]);
  ////tmp_pose = cur_foot_rot.transpose() * Eigen::AngleAxisf(accumulated_yaw_image.at<float>((int)(next_tmp[1]*100), (int)(next_tmp[0]*100)), Eigen::Vector3f::UnitZ()) * tmp_pose;
  //tmp_pose = Eigen::AngleAxisf(accumulated_yaw_image.at<float>((int)(next_tmp[1]*100), (int)(next_tmp[0]*100)), Eigen::Vector3f::UnitZ()) * tmp_pose;
  //tmp_pose = cur_foot_rot.transpose() * tmp_pose;
  //if (0.5 < tmp_pose.norm() && tmp_pose.norm() < 1.5) {
  //  ps.nx =  tmp_pose[0];
  //  ps.ny =  tmp_pose[1];
  //  ps.nz =  tmp_pose[2];
  //} else {
  //  return;
  //}

  //Eigen::Vector3f tmp_pos;
  //tmp_pos = cur_foot_rot.transpose() * (next_foot_pos - cur_foot_pos);
  //tmp_pos[2] = std::abs(tmp_pos[2]) < 0.4 ? tmp_pos[2] : 0;
  //ps.x = tmp_pos(0);
  //ps.y = tmp_pos(1);
  //ps.z = tmp_pos(2);

  //height_publisher_.publish(ps);

  //Eigen::Vector3f start_pos;
  //start_pos = tmp_cur_foot_rot.transpose() * cur_foot_rot * Eigen::Vector3f(ps.x, ps.y, ps.z);
  //Eigen::Vector3f end_pos;
  //end_pos = tmp_cur_foot_rot.transpose() * cur_foot_rot * Eigen::Vector3f(ps.x+0.3*ps.nx, ps.y+0.3*ps.ny, ps.z+0.3*ps.nz);

  //// publish pose msg for visualize
  //visualization_msgs::Marker pose_msg;
  //pose_msg.header = ps.header;
  //pose_msg.ns = "landing_pose";
  //pose_msg.id = 0;
  //pose_msg.lifetime = ros::Duration();
  //pose_msg.type = visualization_msgs::Marker::ARROW;
  //pose_msg.action = visualization_msgs::Marker::ADD;
  //geometry_msgs::Point start;
  //start.x = start_pos(0);
  //start.y = start_pos(1);
  //start.z = start_pos(2);
  //geometry_msgs::Point end;
  //end.x = end_pos(0);
  //end.y = end_pos(1);
  //end.z = end_pos(2);
  //pose_msg.points.push_back(start);
  //pose_msg.points.push_back(end);
  //pose_msg.color.r = 0.0;
  //pose_msg.color.g = 0.8;
  //pose_msg.color.b = 1.0;
  //pose_msg.color.a = 1.0;
  //pose_msg.scale.x = 0.03;
  //pose_msg.scale.y = 0.05;
  //pose_msg.scale.z = 0.07;
  //pose_msg.pose.orientation.w = 1.0;

  //landing_pose_publisher_.publish(pose_msg);
}
}

PLUGINLIB_EXPORT_CLASS(safe_footstep_planner::SteppableRegionPublisherNodelet, nodelet::Nodelet)
