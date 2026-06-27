#include "cogninav_slam_node.hpp"

#include <Eigen/Geometry>

#include <chrono>
#include <sstream>

using std::placeholders::_1;

namespace
{
geometry_msgs::msg::Pose se3fToPose(const Sophus::SE3f & twc)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Vector3f t = twc.translation();
  const Eigen::Quaternionf q(twc.rotationMatrix());
  pose.position.x = t.x();
  pose.position.y = t.y();
  pose.position.z = t.z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}
}  // namespace

CogniNavSlamNode::CogniNavSlamNode()
: Node("cogninav_vslam"),
  clahe_(cv::createCLAHE(3.0, cv::Size(8, 8)))
{
  const std::string default_vocab = "/root/cogninav/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt";
  const std::string default_settings =
    "/root/cogninav/third_party/ORB_SLAM3/Examples/Stereo-Inertial/EuRoC.yaml";

  this->declare_parameter<std::string>("vocabulary", default_vocab);
  this->declare_parameter<std::string>("settings", default_settings);
  this->declare_parameter<std::string>("slam_mode", "stereo_inertial");
  this->declare_parameter<bool>("do_rectify", false);
  this->declare_parameter<bool>("do_equalize", false);
  this->declare_parameter<bool>("publish_tf", true);
  this->declare_parameter<std::string>("frame_id", "map");
  this->declare_parameter<std::string>("camera_frame", "cam0");
  this->declare_parameter<std::string>("left_image_topic", "/cam0/image_raw");
  this->declare_parameter<std::string>("right_image_topic", "/cam1/image_raw");
  this->declare_parameter<std::string>("imu_topic", "/imu0");
  this->declare_parameter<std::string>("odom_topic", "/cogninav/odom");
  this->declare_parameter<std::string>("map_points_topic", "/cogninav/map_points");
  this->declare_parameter<std::string>("trajectory_path", "/tmp/cogninav_trajectory.txt");
  this->declare_parameter<double>("map_publish_hz", 2.0);

  std::string vocabulary;
  std::string settings;
  std::string slam_mode;
  std::string left_topic;
  std::string right_topic;
  std::string imu_topic;
  double map_hz = 2.0;

  this->get_parameter("vocabulary", vocabulary);
  this->get_parameter("settings", settings);
  this->get_parameter("slam_mode", slam_mode);
  this->get_parameter("do_rectify", do_rectify_);
  this->get_parameter("do_equalize", do_equalize_);
  this->get_parameter("publish_tf", publish_tf_);
  this->get_parameter("frame_id", map_frame_);
  this->get_parameter("camera_frame", camera_frame_);
  this->get_parameter("left_image_topic", left_topic);
  this->get_parameter("right_image_topic", right_topic);
  this->get_parameter("imu_topic", imu_topic);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("map_points_topic", map_topic_);
  this->get_parameter("trajectory_path", trajectory_path_);
  this->get_parameter("map_publish_hz", map_hz);

  use_imu_ = (slam_mode == "stereo_inertial" || slam_mode == "imu_stereo");
  ORB_SLAM3::System::eSensor sensor = use_imu_ ?
    ORB_SLAM3::System::IMU_STEREO :
    ORB_SLAM3::System::STEREO;

  RCLCPP_INFO(this->get_logger(), "Starting ORB-SLAM3 (%s)", slam_mode.c_str());
  RCLCPP_INFO(this->get_logger(), "  vocab: %s", vocabulary.c_str());
  RCLCPP_INFO(this->get_logger(), "  settings: %s", settings.c_str());

  slam_ = std::make_unique<ORB_SLAM3::System>(vocabulary, settings, sensor, false);

  if (do_rectify_) {
    cv::FileStorage fs(settings, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open settings for rectify: %s", settings.c_str());
      throw std::runtime_error("settings file missing");
    }

    cv::Mat k_l, k_r, p_l, p_r, r_l, r_r, d_l, d_r;
    fs["LEFT.K"] >> k_l;
    fs["RIGHT.K"] >> k_r;
    fs["LEFT.P"] >> p_l;
    fs["RIGHT.P"] >> p_r;
    fs["LEFT.R"] >> r_l;
    fs["RIGHT.R"] >> r_r;
    fs["LEFT.D"] >> d_l;
    fs["RIGHT.D"] >> d_r;
    const int rows_l = fs["LEFT.height"];
    const int cols_l = fs["LEFT.width"];
    const int rows_r = fs["RIGHT.height"];
    const int cols_r = fs["RIGHT.width"];

    cv::initUndistortRectifyMap(
      k_l, d_l, r_l, p_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, m1l_, m2l_);
    cv::initUndistortRectifyMap(
      k_r, d_r, r_r, p_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, m1r_, m2r_);
  }

  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(map_topic_, 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  sub_left_ = this->create_subscription<sensor_msgs::msg::Image>(
    left_topic, rclcpp::SensorDataQoS(), std::bind(&CogniNavSlamNode::grabImageLeft, this, _1));
  sub_right_ = this->create_subscription<sensor_msgs::msg::Image>(
    right_topic, rclcpp::SensorDataQoS(), std::bind(&CogniNavSlamNode::grabImageRight, this, _1));

  if (use_imu_) {
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(), std::bind(&CogniNavSlamNode::grabImu, this, _1));
    sync_thread_ = std::thread(&CogniNavSlamNode::syncWithImu, this);
  }

  if (map_hz > 0.0) {
    map_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / map_hz),
      std::bind(&CogniNavSlamNode::publishMapPoints, this));
  }

  RCLCPP_INFO(
    this->get_logger(), "Subscribed L=%s R=%s IMU=%s", left_topic.c_str(), right_topic.c_str(),
    use_imu_ ? imu_topic.c_str() : "(disabled)");
}

CogniNavSlamNode::~CogniNavSlamNode()
{
  running_ = false;
  if (sync_thread_.joinable()) {
    sync_thread_.join();
  }
  if (slam_) {
    slam_->Shutdown();
    slam_->SaveTrajectoryTUM(trajectory_path_);
    RCLCPP_INFO(this->get_logger(), "Saved trajectory to %s", trajectory_path_.c_str());
  }
}

void CogniNavSlamNode::grabImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(buf_mutex_);
  imu_buf_.push(msg);
}

void CogniNavSlamNode::grabImageLeft(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(buf_mutex_left_);
  if (!img_left_buf_.empty()) {
    img_left_buf_.pop();
  }
  img_left_buf_.push(msg);
}

void CogniNavSlamNode::grabImageRight(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(buf_mutex_right_);
  if (!img_right_buf_.empty()) {
    img_right_buf_.pop();
  }
  img_right_buf_.push(msg);
}

cv::Mat CogniNavSlamNode::getImage(const sensor_msgs::msg::Image::SharedPtr & msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
    return {};
  }
  return cv_ptr->image.clone();
}

void CogniNavSlamNode::publishPose(const Sophus::SE3f & twc, const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = map_frame_;
  odom.child_frame_id = camera_frame_;
  odom.pose.pose = se3fToPose(twc);
  pub_odom_->publish(odom);

  if (publish_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom.header;
    tf_msg.child_frame_id = camera_frame_;
    tf_msg.transform.translation.x = odom.pose.pose.position.x;
    tf_msg.transform.translation.y = odom.pose.pose.position.y;
    tf_msg.transform.translation.z = odom.pose.pose.position.z;
    tf_msg.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }
}

void CogniNavSlamNode::publishMapPoints()
{
  if (!slam_) {
    return;
  }

  const auto map_points = slam_->GetTrackedMapPoints();
  if (map_points.empty()) {
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = this->now();
  cloud.header.frame_id = map_frame_;
  cloud.height = 1;
  cloud.width = 0;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  std::vector<float> xyz;
  xyz.reserve(map_points.size() * 3);
  for (auto * mp : map_points) {
    if (!mp || mp->isBad()) {
      continue;
    }
    const Eigen::Vector3f pos = mp->GetWorldPos();
    xyz.push_back(pos.x());
    xyz.push_back(pos.y());
    xyz.push_back(pos.z());
  }

  if (xyz.empty()) {
    return;
  }

  cloud.width = static_cast<uint32_t>(xyz.size() / 3);
  modifier.resize(xyz.size() / 3);
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  for (size_t i = 0; i < xyz.size(); i += 3) {
    *iter_x = xyz[i];
    *iter_y = xyz[i + 1];
    *iter_z = xyz[i + 2];
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  pub_map_->publish(cloud);
}

void CogniNavSlamNode::syncWithImu()
{
  const double max_time_diff = 0.01;

  while (running_ && rclcpp::ok()) {
    cv::Mat im_left;
    cv::Mat im_right;
    double t_left = 0.0;
    rclcpp::Time stamp;
    std::vector<ORB_SLAM3::IMU::Point> imu_meas;
    bool ready = false;

    if (img_left_buf_.empty() || img_right_buf_.empty() || imu_buf_.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    buf_mutex_left_.lock();
    buf_mutex_right_.lock();
    t_left = Utility::StampToSec(img_left_buf_.front()->header.stamp);
    double t_right = Utility::StampToSec(img_right_buf_.front()->header.stamp);

    while ((t_left - t_right) > max_time_diff && img_right_buf_.size() > 1) {
      img_right_buf_.pop();
      t_right = Utility::StampToSec(img_right_buf_.front()->header.stamp);
    }
    while ((t_right - t_left) > max_time_diff && img_left_buf_.size() > 1) {
      img_left_buf_.pop();
      t_left = Utility::StampToSec(img_left_buf_.front()->header.stamp);
    }

    if (std::abs(t_left - t_right) <= max_time_diff) {
      buf_mutex_.lock();
      if (!imu_buf_.empty() && t_left <= Utility::StampToSec(imu_buf_.back()->header.stamp)) {
        im_left = getImage(img_left_buf_.front());
        im_right = getImage(img_right_buf_.front());
        stamp = img_left_buf_.front()->header.stamp;
        img_left_buf_.pop();
        img_right_buf_.pop();

        while (!imu_buf_.empty() && Utility::StampToSec(imu_buf_.front()->header.stamp) <= t_left) {
          const auto & imu = imu_buf_.front();
          const double t = Utility::StampToSec(imu->header.stamp);
          imu_meas.emplace_back(
            cv::Point3f(
              imu->linear_acceleration.x, imu->linear_acceleration.y,
              imu->linear_acceleration.z),
            cv::Point3f(
              imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z),
            t);
          imu_buf_.pop();
        }
        ready = !im_left.empty() && !im_right.empty();
      }
      buf_mutex_.unlock();
    }
    buf_mutex_right_.unlock();
    buf_mutex_left_.unlock();

    if (!ready) {
      continue;
    }

    if (do_equalize_) {
      clahe_->apply(im_left, im_left);
      clahe_->apply(im_right, im_right);
    }
    if (do_rectify_) {
      cv::remap(im_left, im_left, m1l_, m2l_, cv::INTER_LINEAR);
      cv::remap(im_right, im_right, m1r_, m2r_, cv::INTER_LINEAR);
    }

    const Sophus::SE3f tcw = slam_->TrackStereo(im_left, im_right, t_left, imu_meas);
    publishPose(tcw.inverse(), stamp);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
