#ifndef COGNINAV_SLAM_NODE_HPP_
#define COGNINAV_SLAM_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>

#include <cogninav/cv_bridge_compat.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>

#include "System.h"
#include "utility.hpp"

class CogniNavSlamNode : public rclcpp::Node
{
public:
  CogniNavSlamNode();
  ~CogniNavSlamNode() override;

private:
  void grabImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void grabImageLeft(const sensor_msgs::msg::Image::SharedPtr msg);
  void grabImageRight(const sensor_msgs::msg::Image::SharedPtr msg);
  void grabDynamicMask(const sensor_msgs::msg::Image::SharedPtr msg);
  cv::Mat getImage(const sensor_msgs::msg::Image::SharedPtr & msg);
  void applyDynamicMask(cv::Mat & left, cv::Mat & right);
  void syncWithImu();
  void syncStereo();
  void publishMapPoints();
  void publishPose(const Sophus::SE3f & twc, const rclcpp::Time & stamp);

  std::unique_ptr<ORB_SLAM3::System> slam_;
  std::thread sync_thread_;
  std::atomic<bool> running_{true};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_left_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_right_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_dynamic_mask_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_mask_stats_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr map_timer_;

  std::mutex buf_mutex_;
  std::mutex buf_mutex_left_;
  std::mutex buf_mutex_right_;
  std::mutex mask_mutex_;
  std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf_;
  std::queue<sensor_msgs::msg::Image::SharedPtr> img_left_buf_;
  std::queue<sensor_msgs::msg::Image::SharedPtr> img_right_buf_;
  cv::Mat latest_mask_;

  bool do_rectify_{false};
  bool do_equalize_{false};
  bool publish_tf_{true};
  bool use_imu_{true};
  bool use_dynamic_mask_{false};
  cv::Mat m1l_, m2l_, m1r_, m2r_;
  cv::Ptr<cv::CLAHE> clahe_;

  std::string map_frame_;
  std::string camera_frame_;
  std::string odom_topic_;
  std::string map_topic_;
  std::string trajectory_path_;
  size_t max_map_points_publish_{50000};
  size_t lost_map_frames_{0};
  int process_every_n_{1};
  size_t stereo_frame_counter_{0};
  std::unordered_map<unsigned long, Eigen::Vector3f> map_points_cache_;

  static constexpr size_t kMaxImageBuffer = 30;
  static constexpr size_t kMaxImuBuffer = 500;
};

#endif  // COGNINAV_SLAM_NODE_HPP_
