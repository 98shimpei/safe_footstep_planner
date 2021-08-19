#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <tf/transform_listener.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <jsk_recognition_utils/geo_util.h>
#include <safe_footstep_planner/OnlineFootStep.h>
#include <safe_footstep_planner/safe_footstep_util.h>

class TargetHeightPublisher
{
public:
    TargetHeightPublisher();
    ~TargetHeightPublisher(){};

private:
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void targetCallback(const safe_footstep_planner::OnlineFootStep::ConstPtr& msg);
    void matrixTFToEigen(const tf::Matrix3x3 &t, Eigen::Matrix3f &e);
    void vectorTFToEigen(const tf::Vector3& t, Eigen::Vector3f& k);
    void calcFootRotFromNormal (Eigen::Matrix3f& foot_rot, const Eigen::Matrix3f& orig_rot, const Eigen::Vector3f& n);
    Eigen::Vector3f rpyFromRot(const Eigen::Matrix3f& m);
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher height_publisher_;
    ros::Publisher landing_pose_publisher_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber target_sub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    tf::TransformListener listener_;
};

TargetHeightPublisher::TargetHeightPublisher() : nh_(""), pnh_("~")
{
    height_publisher_ = nh_.advertise<safe_footstep_planner::OnlineFootStep>("landing_height", 1);
    landing_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("landing_pose", 1);
    // cloud_sub_ = nh_.subscribe("rt_accumulated_heightmap_pointcloud/output", 1, &TargetHeightPublisher::pointcloudCallback, this);
    cloud_sub_ = nh_.subscribe("rt_accumulated_heightmap_pointcloud_odomrelative/output", 1, &TargetHeightPublisher::pointcloudCallback, this);
    // cloud_sub_ = nh_.subscribe("rt_accumulated_heightmap_pointcloud_odomrelative_fixed/output", 1, &TargetHeightPublisher::pointcloudCallback, this);
    target_sub_ = nh_.subscribe("landing_target", 1, &TargetHeightPublisher::targetCallback, this);
    // std::cerr << "TargetHeihgtPublisher Initialized!!!!!! " << std::endl;
}

void TargetHeightPublisher::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_);
    // std::cout << "cloud size : " << cloud_->size() << std::endl;
}

void TargetHeightPublisher::targetCallback(const safe_footstep_planner::OnlineFootStep::ConstPtr& msg)
{
    double px = msg->x;
    double py = msg->y;
    double pz = msg->z;
    // std::cerr << "hoge: " << std::endl;
    // transform
    std::string target_frame;
    if (msg->l_r) {
        target_frame = "/lleg_end_coords";
    }
    else {
        target_frame = "/rleg_end_coords";
    }

    tf::StampedTransform transform;
    // listener_.lookupTransform("/body_on_odom", target_frame, ros::Time(0), transform); // map relative to target_frame
    // listener_.lookupTransform("/odom_ground", target_frame, ros::Time(0), transform); // map relative to target_frame
    listener_.lookupTransform(cloud_->header.frame_id, target_frame, ros::Time(0), transform); // map relative to target_frame
    Eigen::Vector3f cur_foot_pos, ez(Eigen::Vector3f::UnitZ());
    safe_footstep_util::vectorTFToEigen(transform.getOrigin(), cur_foot_pos);
    Eigen::Matrix3f tmp_cur_foot_rot, cur_foot_rot;
    safe_footstep_util::matrixTFToEigen(transform.getBasis(), tmp_cur_foot_rot);
    safe_footstep_util::calcFootRotFromNormal(cur_foot_rot, tmp_cur_foot_rot, ez);
    Eigen::Vector3f next_foot_pos;
    next_foot_pos = cur_foot_pos + cur_foot_rot * Eigen::Vector3f(px, py, pz);

    // double threshold = 0.04;
    double threshold = 0.03;
    // double cur_az = 0.0, next_az = 0.0;
    double cur_az_front = 0.0, next_az_front = 0.0;
    double cur_az_rear = 0.0, next_az_rear = 0.0;
    // int count_cur = 0, count_next = 0;
    int count_cur_front = 0, count_next_front = 0;
    int count_cur_rear = 0, count_next_rear = 0;
    std::vector<double>  next_az_vec, cur_az_vec;
    std::vector<double>  next_az_vec_front, cur_az_vec_front;
    std::vector<double>  next_az_vec_rear, cur_az_vec_rear;
    pcl::PointXYZ pp;
    // Eigen::Vector3f pos_margin (0.05, 0, 0);
    Eigen::Vector3f pos_margin_front (0.04, 0, 0);
    Eigen::Vector3f pos_margin_rear (0.07, 0, 0);
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);

    // TODO rotation should be considered!

    if (cloud_) {
        for (int i = 0; i < cloud_->size(); i++) {
            pp = cloud_->points[i];
            // calc front mean height
            if (std::fabs(pp.x - (next_foot_pos(0) + pos_margin_front(0))) < threshold &&
                std::fabs(pp.y - next_foot_pos(1)) < threshold) {
                next_az_front += pp.z;
                count_next_front++;
                // TODO indices should be considered!
                indices->indices.push_back(i);
                next_az_vec_front.push_back(pp.z);
            }
            if (std::fabs(pp.x - (cur_foot_pos(0) + pos_margin_front(0))) < threshold &&
                std::fabs(pp.y - cur_foot_pos(1)) < threshold) {
                cur_az_front += pp.z;
                count_cur_front++;
                cur_az_vec_front.push_back(pp.z);
            }
            // calc rear mean height
            if (std::fabs(pp.x - (next_foot_pos(0) - pos_margin_rear(0))) < threshold &&
                std::fabs(pp.y - next_foot_pos(1)) < threshold) {
                next_az_rear += pp.z;
                count_next_rear++;
                // TODO indices should be considered!
                indices->indices.push_back(i);
                next_az_vec_rear.push_back(pp.z);
            }
            if (std::fabs(pp.x - (cur_foot_pos(0) - pos_margin_rear(0))) < threshold &&
                std::fabs(pp.y - cur_foot_pos(1)) < threshold) {
                cur_az_rear += pp.z;
                count_cur_rear++;
                cur_az_vec_rear.push_back(pp.z);
            }
        }
        // ROS_INFO("x: %f  y: %f  z: %f,  rel_pos x: %f  rel_pos y: %f, az/count: %f", landing_pos.getX(), landing_pos.getY(), landing_pos.getZ(), rel_landing_pos.getX(), rel_landing_pos.getY(), az / count);

        // publish point
        if (count_next_front + count_next_rear > 0 && count_cur_front + count_cur_rear > 0) {
            safe_footstep_planner::OnlineFootStep ps;
            std_msgs::Header header;
            header.frame_id = target_frame.substr(1, target_frame.length() - 1);
            // header.stamp = ros::Time::now();
            header.stamp = ros::Time(0);
            ps.header = header;

            // mean
            // cur_foot_pos(2) = cur_az / static_cast<double>(count_cur);
            // next_foot_pos(2) = next_az / static_cast<double>(count_next);
            // median
            std::sort(next_az_vec_front.begin(), next_az_vec_front.end());
            std::sort(cur_az_vec_front.begin(), cur_az_vec_front.end());
            std::sort(next_az_vec_rear.begin(), next_az_vec_rear.end());
            std::sort(cur_az_vec_rear.begin(), cur_az_vec_rear.end());

            cur_foot_pos(2) = std::max(cur_az_vec_front[cur_az_vec_front.size()/2],
                                       cur_az_vec_rear[cur_az_vec_rear.size()/2]);
            // omori comment out on 2020/12/24
            // cur_foot_pos(2) = (std::max(cur_az_vec_front[cur_az_vec_front.size()/2],
            //                             cur_az_vec_rear[cur_az_vec_rear.size()/2]) * 2
            //                    + cur_foot_pos(2)) / 3;
            next_foot_pos(2) = std::max(next_az_vec_front[next_az_vec_front.size()/2],
                                        next_az_vec_rear[next_az_vec_rear.size()/2]);

            Eigen::Vector3f tmp_pos;
            tmp_pos = cur_foot_rot.transpose() * (next_foot_pos - cur_foot_pos);
            ps.x = tmp_pos(0);
            ps.y = tmp_pos(1);
            // target height range is -0.2 <= target_height <= 0.2


            if (std::abs(tmp_pos(2)) >= 0.2 || !std::isfinite(tmp_pos(2))) return;
            // double limited_h = std::min(0.2, std::max(-0.2,static_cast<double>(tmp_pos(2))));
            // ps.z = limited_h;

            ps.z = tmp_pos(2);
            ps.l_r = msg->l_r;
            // std::cerr << "height: " << limited_h << std::endl;

            // estimage plane by RANSAC
            // int minimun_indices = 10;
            // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            // pcl::SACSegmentation<pcl::PointXYZ> seg;
            // seg.setOptimizeCoefficients (true);
            // seg.setRadiusLimits(0.01, std::numeric_limits<double>::max ());
            // seg.setMethodType(pcl::SAC_RANSAC);
            // seg.setDistanceThreshold(0.1);
            // seg.setModelType(pcl::SACMODEL_PLANE);
            // seg.setInputCloud(cloud_);
            // //
            // seg.setIndices(indices);
            // seg.setMaxIterations(100);
            // seg.segment(*inliers, *coefficients);

            // if (inliers->indices.size() == 0) {
            //     std::cerr <<  " no plane" << std::endl;
            // }
            // else if (inliers->indices.size() < minimun_indices) {
            //     std::cerr <<  " no enough inliners " << inliers->indices.size() <<  std::endl;
            // }
            // else {
            //     jsk_recognition_utils::Plane plane(coefficients->values);
            //     if (!plane.isSameDirection(ez)) {
            //         plane = plane.flip();
            //     }
            //     Eigen::Vector3f next_n = plane.getNormal();
            //     next_n = cur_foot_rot.transpose() * next_n; // cur_foot relative

            //     // ps.nx =  next_n(0);
            //     // ps.ny =  next_n(1);
            //     // ps.nz =  next_n(2);
            //     ps.nx =  0;
            //     ps.ny =  0;
            //     ps.nz =  1;
            // }
            // ======= omori add 2020/02/16 ===========
            ps.nx =  0;
            ps.ny =  0;
            ps.nz =  1;
            // ========================================
            height_publisher_.publish(ps);

            // publish pose msg for visualize
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header = ps.header;
            pose_msg.pose.position.x = ps.x;
            pose_msg.pose.position.y = ps.y;
            pose_msg.pose.position.z = ps.z;
            pose_msg.pose.orientation.x = 0;
            pose_msg.pose.orientation.y = -0.7071068;
            pose_msg.pose.orientation.z = 0;
            pose_msg.pose.orientation.w = 0.7071068;

            landing_pose_publisher_.publish(pose_msg);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_height_publisher");
    TargetHeightPublisher target_height_publisher;
    ros::Duration(5);
    ros::spin();

    return 0;
}
