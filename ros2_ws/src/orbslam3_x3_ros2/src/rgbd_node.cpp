// SPDX-License-Identifier: GPL-3.0-or-later
// X3-specific implementation; reference architectures are listed in config/orbslam3_ros2_references.md.
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include "image_conversion.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "System.h"

namespace fs = std::filesystem;
using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;

class RGBDNode : public rclcpp::Node {
 public:
  RGBDNode() : Node("orbslam3_x3_rgbd") {
    vocabulary_ = declare_parameter<std::string>("vocabulary", "");
    output_ = declare_parameter<std::string>("output", "");
    viewer_ = declare_parameter<bool>("viewer", true);
    save_atlas_ = declare_parameter<bool>("save_atlas", false);
    fps_ = declare_parameter<double>("camera_fps", 30.0);
    const auto profile = declare_parameter<std::string>("input_profile", "real");
    if (profile != "real")
      throw std::runtime_error("input_profile must be real");
    real_input_ = profile == "real";
    const auto sync_mode = declare_parameter<std::string>("sync_mode", real_input_ ? "approximate" : "exact");
    max_sync_interval_ms_ = declare_parameter<double>("max_sync_interval_ms", 20.0);
    depth_scale_ = declare_parameter<double>("depth_scale", 0.001);
    if ((sync_mode != "exact" && sync_mode != "approximate") ||
        !std::isfinite(max_sync_interval_ms_) || max_sync_interval_ms_ <= 0 || max_sync_interval_ms_ > 100 ||
        !std::isfinite(depth_scale_) || depth_scale_ <= 0)
      throw std::runtime_error("Require exact/approximate sync, interval in (0,100] ms and positive depth_scale");
    if (!fs::is_regular_file(vocabulary_) || output_.empty() ||
        !std::isfinite(fps_) || fps_ <= 0 || fps_ > 120 || std::floor(fps_) != fps_)
      throw std::runtime_error("Require existing vocabulary, output directory and valid camera_fps");
    fs::create_directories(output_);
    for (const auto *name : {"camera.yaml", "frames.csv", "summary.txt", "input_timestamps.csv",
                             "KeyFrameTrajectory.txt", "CameraTrajectory.txt"})
      if (fs::exists(fs::path(output_) / name))
        throw std::runtime_error("Output already contains SLAM files; use a fresh directory");
    inputs_.open(fs::path(output_) / "input_timestamps.csv");
    if (!inputs_) throw std::runtime_error("Cannot write input timestamp ledger");
    inputs_ << "timestamp_ns,action\n";
    const auto rgb_topic = declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
    const auto depth_topic = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    const auto info_topic = declare_parameter<std::string>("info_topic", "/camera/color/camera_info");
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/orbslam3/camera_pose", 10);
    state_pub_ = create_publisher<std_msgs::msg::Int32>("/orbslam3/tracking_state", 10);
    info_sub_ = create_subscription<Info>(info_topic, rclcpp::SensorDataQoS(),
      [this](Info::ConstSharedPtr info) { camera_info(info); });
    rgb_.subscribe(this, rgb_topic, rmw_qos_profile_sensor_data);
    depth_.subscribe(this, depth_topic, rmw_qos_profile_sensor_data);
    rgb_.registerCallback([this](const Image::ConstSharedPtr &) { ++rgb_received_; });
    depth_.registerCallback([this](const Image::ConstSharedPtr &) { ++depth_received_; });
    if (sync_mode == "exact") {
      sync_ = std::make_unique<Sync>(Policy(5), rgb_, depth_);
      sync_->registerCallback(&RGBDNode::receive, this);
    } else {
      ApproxPolicy policy(10);
      policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_sync_interval_ms_ / 1000.0));
      approx_sync_ = std::make_unique<ApproxSync>(policy);
      approx_sync_->connectInput(rgb_, depth_);
      approx_sync_->registerCallback(&RGBDNode::receive, this);
    }
    timer_ = create_wall_timer(std::chrono::seconds(3), [this]() {
      RCLCPP_INFO(get_logger(), "%s rgb=%lu depth=%lu paired=%lu processed=%lu tracked=%lu rejected=%lu overwritten=%lu state=%d",
        ready_ ? "RUNNING" : "WAITING_FOR_CAMERA_OR_LOADING_VOCABULARY",
        rgb_received_.load(), depth_received_.load(), paired_.load(), processed_.load(),
        tracked_.load(), rejected_.load(), overwritten_.load(), state_.load());
    });
    worker_ = std::thread(&RGBDNode::run, this);
    RCLCPP_INFO(get_logger(), "Waiting for CameraInfo and %s RGB-D pairs; profile=%s max_interval_ms=%.1f output=%s",
      sync_mode.c_str(), profile.c_str(), max_sync_interval_ms_, output_.c_str());
  }

  ~RGBDNode() override { finish(); }

  void finish() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      pending_rgb_.reset(); pending_depth_.reset();
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    inputs_.flush();
  }
  bool failed() const { return failed_; }

 private:
  using Policy = message_filters::sync_policies::ExactTime<Image, Image>;
  using Sync = message_filters::Synchronizer<Policy>;
  using ApproxPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  using ApproxSync = message_filters::Synchronizer<ApproxPolicy>;
  void receive(Image::ConstSharedPtr rgb, Image::ConstSharedPtr depth) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++paired_;
    if (stopping_) return;
    inputs_ << rclcpp::Time(rgb->header.stamp).nanoseconds() << ','
            << (info_ ? "queued" : "before_info") << '\n';
    if (!info_) { ++before_info_; return; }
    if (pending_rgb_) ++overwritten_;
    pending_rgb_ = rgb;
    pending_depth_ = depth;
    cv_.notify_one();
  }
  void camera_info(Info::ConstSharedPtr info) {
    bool valid = info->width && info->height && info->k[0] > 0 && info->k[4] > 0 &&
      !info->header.frame_id.empty() &&
      (info->distortion_model.empty() || info->distortion_model == "plumb_bob");
    for (double value : info->k) valid = valid && std::isfinite(value);
    for (double value : info->d)
      valid = valid && std::isfinite(value) && (real_input_ || std::abs(value) < 1e-9);
    if (real_input_)
      valid = valid && (info->d.empty() || (info->d.size() == 5 && info->distortion_model == "plumb_bob"));
    if (!valid) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
        "Require finite calibrated pinhole CameraInfo with empty distortion or plumb_bob D[5]");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (info_ && (info_->width != info->width || info_->height != info->height ||
                  info_->k != info->k || info_->d != info->d ||
                  info_->header.frame_id != info->header.frame_id)) {
      failed_ = true;
      RCLCPP_ERROR(get_logger(), "Camera calibration changed; restart SLAM for the new camera");
      rclcpp::shutdown();
      return;
    }
    info_ = info;
  }

  void settings(const Info &info) {
    // OpenCV's YAML writer rejects dots in keys, although its reader accepts
    // ORB-SLAM3's flat dotted-key format. Emit that format explicitly.
    std::ofstream yaml(fs::path(output_) / "camera.yaml");
    if (!yaml) throw std::runtime_error("Cannot write camera.yaml");
    yaml << "%YAML:1.0\nFile.version: \"1.0\"\nCamera.type: \"PinHole\"\n";
    yaml << std::scientific << std::setprecision(17);
    const auto real = [&yaml](const char *key, double value) { yaml << key << ": " << value << '\n'; };
    real("Camera1.fx", info.k[0]); real("Camera1.fy", info.k[4]);
    real("Camera1.cx", info.k[2]); real("Camera1.cy", info.k[5]);
    size_t distortion_index = 0;
    for (const auto *key : {"Camera1.k1", "Camera1.k2", "Camera1.p1", "Camera1.p2", "Camera1.k3"}) {
      real(key, real_input_ && distortion_index < info.d.size() ? info.d[distortion_index] : 0.0);
      ++distortion_index;
    }
    yaml << "Camera.width: " << info.width << "\nCamera.height: " << info.height << "\nCamera.RGB: 1\n";
    yaml << "Camera.fps: " << static_cast<int>(fps_) << '\n';
    if (save_atlas_) yaml << "System.SaveAtlasToFile: \"Atlas\"\n";
    // b * ThDepth = 4 m is an algorithm close-point threshold, not a physical stereo baseline.
    real("Stereo.b", 0.1); real("Stereo.ThDepth", 40.0); real("RGBD.DepthMapFactor", 1.0);
    yaml << "ORBextractor.nFeatures: 1000\nORBextractor.nLevels: 8\n"
         << "ORBextractor.iniThFAST: 15\nORBextractor.minThFAST: 5\n";
    real("ORBextractor.scaleFactor", 1.2);
    real("Viewer.KeyFrameSize", 0.05); real("Viewer.KeyFrameLineWidth", 1.0);
    real("Viewer.GraphLineWidth", 0.9); real("Viewer.PointSize", 2.0);
    real("Viewer.CameraSize", 0.08); real("Viewer.CameraLineWidth", 3.0);
    real("Viewer.ViewpointX", 0.0); real("Viewer.ViewpointY", -0.7);
    real("Viewer.ViewpointZ", -1.8); real("Viewer.ViewpointF", 500.0);
    yaml.flush();
    if (!yaml) throw std::runtime_error("Failed writing camera.yaml");
  }

  void run() {
    std::unique_ptr<ORB_SLAM3::System> slam;
    try {
      std::ofstream frames(fs::path(output_) / "frames.csv");
      frames << "timestamp,tracking_state,valid_depth_fraction,timestamp_ns,rgb_depth_dt_ms\n";
      double last_stamp = -1;
      while (true) {
        Image::ConstSharedPtr rgb, depth;
        Info::ConstSharedPtr info;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          cv_.wait(lock, [this]() { return stopping_ || pending_rgb_; });
          if (stopping_) break;
          rgb.swap(pending_rgb_); depth.swap(pending_depth_); info = info_;
        }
        try {
          const double stamp = rclcpp::Time(rgb->header.stamp).seconds();
          const double dt_ms = static_cast<double>(rclcpp::Time(depth->header.stamp).nanoseconds() -
                                                  rclcpp::Time(rgb->header.stamp).nanoseconds()) / 1e6;
          const bool integer_depth = real_input_ && depth->encoding == "16UC1";
          if (stamp <= last_stamp || rgb->width != info->width || rgb->height != info->height ||
              depth->width != rgb->width || depth->height != rgb->height ||
              rgb->header.frame_id != info->header.frame_id ||
              depth->header.frame_id != rgb->header.frame_id ||
              (depth->encoding != "32FC1" && !integer_depth) ||
              std::abs(dt_ms) > max_sync_interval_ms_ ||
              (rgb->encoding != "rgb8" && rgb->encoding != "bgr8"))
            throw std::runtime_error("Require increasing stamp, registered sizes/frames, bounded time difference, rgb8/bgr8 and supported depth");
          cv::Mat color = x3_rgbd::color_rgb(*rgb);
          cv::Mat range = x3_rgbd::depth_metres(*depth, real_input_, depth_scale_);
          size_t valid = 0;
          for (int y = 0; y < range.rows; ++y) {
            float *row = range.ptr<float>(y);
            for (int x = 0; x < range.cols; ++x) {
              if (std::isfinite(row[x]) && row[x] > 0) ++valid;
              else row[x] = 0;
            }
          }
          if (!valid) throw std::runtime_error("Depth image has no valid positive samples");
          if (!slam) {
            settings(*info);
            RCLCPP_INFO(get_logger(), "Camera %ux%u fx=%.3f fy=%.3f cx=%.3f cy=%.3f depth=metres; loading vocabulary",
              info->width, info->height, info->k[0], info->k[4], info->k[2], info->k[5]);
            slam = std::make_unique<ORB_SLAM3::System>(vocabulary_,
              (fs::path(output_) / "camera.yaml").string(), ORB_SLAM3::System::RGBD, viewer_);
            ready_ = true;
            RCLCPP_INFO(get_logger(), "X3_RGBD_READY: online TrackRGBD active");
            // Drop the pair held during vocabulary loading and use the latest live pair.
            continue;
          }
          const auto Tcw = slam->TrackRGBD(color, range, stamp);
          last_stamp = stamp;
          ++processed_;
          state_ = slam->GetTrackingState();
          std_msgs::msg::Int32 state_msg; state_msg.data = state_;
          if (rclcpp::ok()) state_pub_->publish(state_msg);
          frames << std::setprecision(15) << stamp << ',' << state_ << ','
                 << static_cast<double>(valid) / range.total() << ','
                 << rclcpp::Time(rgb->header.stamp).nanoseconds() << ',' << dt_ms << '\n';
          if (state_ == ORB_SLAM3::Tracking::OK) {
            ++tracked_;
            const auto Twc = Tcw.inverse();
            const auto t = Twc.translation(); const auto q = Twc.unit_quaternion();
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = rgb->header.stamp; pose.header.frame_id = "slam_map";
            pose.pose.position.x = t.x(); pose.pose.position.y = t.y(); pose.pose.position.z = t.z();
            pose.pose.orientation.x = q.x(); pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z(); pose.pose.orientation.w = q.w();
            if (rclcpp::ok()) pose_pub_->publish(pose);
          }
        } catch (const cv::Exception &error) {
          ++rejected_; RCLCPP_ERROR(get_logger(), "Image rejected: %s", error.what());
        } catch (const std::runtime_error &error) {
          ++rejected_;
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "Pair rejected: %s", error.what());
        }
      }
    } catch (const std::exception &error) {
      failed_ = true;
      RCLCPP_ERROR(get_logger(), "SLAM worker failed: %s", error.what());
      rclcpp::shutdown();
    }
    if (slam) {
      slam->Shutdown();
      const auto kf = (fs::path(output_) / "KeyFrameTrajectory.txt").string();
      slam->SaveKeyFrameTrajectoryTUM(kf);
      if (fs::exists(kf) && fs::file_size(kf))
        slam->SaveTrajectoryTUM((fs::path(output_) / "CameraTrajectory.txt").string());
    }
    std::ofstream summary(fs::path(output_) / "summary.txt");
    summary << "processed=" << processed_ << "\ntracked=" << tracked_ << "\npaired=" << paired_
            << "\nrejected=" << rejected_ << "\noverwritten=" << overwritten_
            << "\nbefore_info=" << before_info_ << "\nfailed=" << failed_ << '\n';
    RCLCPP_INFO(get_logger(), "X3_RGBD_SAVED: processed=%lu tracked=%lu output=%s",
      processed_.load(), tracked_.load(), output_.c_str());
  }

  std::string vocabulary_, output_;
  std::ofstream inputs_;
  bool viewer_, save_atlas_;
  bool real_input_ = false;
  double fps_, max_sync_interval_ms_, depth_scale_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool stopping_ = false;
  Info::ConstSharedPtr info_;
  Image::ConstSharedPtr pending_rgb_, pending_depth_;
  std::thread worker_;
  std::atomic<bool> ready_{false}, failed_{false};
  std::atomic<unsigned long> rgb_received_{0}, depth_received_{0}, paired_{0}, processed_{0},
    tracked_{0}, rejected_{0}, overwritten_{0}, before_info_{0};
  std::atomic<int> state_{-1};
  rclcpp::Subscription<Info>::SharedPtr info_sub_;
  message_filters::Subscriber<Image> rgb_, depth_;
  std::unique_ptr<Sync> sync_;
  std::unique_ptr<ApproxSync> approx_sync_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<RGBDNode>();
    rclcpp::spin(node);
    node->finish();
    const bool failed = node->failed();
    node.reset();
    if (rclcpp::ok()) rclcpp::shutdown();
    return failed ? 1 : 0;
  } catch (const std::exception &error) {
    std::cerr << "X3 RGB-D failed: " << error.what() << std::endl;
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
