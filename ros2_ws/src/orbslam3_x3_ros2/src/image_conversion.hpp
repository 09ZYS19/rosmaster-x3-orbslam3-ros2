// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace x3_rgbd {
// Copy into aligned, owned OpenCV storage; honor ROS row padding and byte order.
inline cv::Mat copy_pixels(const sensor_msgs::msg::Image &message, int type) {
  if (!message.width || !message.height ||
      message.width > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
      message.height > static_cast<uint32_t>(std::numeric_limits<int>::max()))
    throw std::runtime_error("Invalid image dimensions");
  const uint64_t row_bytes = static_cast<uint64_t>(message.width) * CV_ELEM_SIZE(type);
  const uint64_t storage_bytes = static_cast<uint64_t>(message.step) * message.height;
  if (message.step < row_bytes || storage_bytes > message.data.size())
    throw std::runtime_error("Image step/data size does not match its dimensions and encoding");
  cv::Mat image(static_cast<int>(message.height), static_cast<int>(message.width), type);
  for (uint32_t y = 0; y < message.height; ++y)
    std::memcpy(image.ptr(static_cast<int>(y)),
                message.data.data() + static_cast<size_t>(y) * message.step,
                static_cast<size_t>(row_bytes));
  const uint16_t marker = 0x0102;
  const bool host_big_endian = *reinterpret_cast<const uint8_t *>(&marker) == 0x01;
  const size_t scalar_bytes = image.elemSize1();
  if (scalar_bytes > 1 && static_cast<bool>(message.is_bigendian) != host_big_endian) {
    for (int y = 0; y < image.rows; ++y) {
      auto *row = image.ptr<uint8_t>(y);
      for (size_t offset = 0; offset < row_bytes; offset += scalar_bytes)
        std::reverse(row + offset, row + offset + scalar_bytes);
    }
  }
  return image;
}

inline cv::Mat color_rgb(const sensor_msgs::msg::Image &message) {
  if (message.encoding != "rgb8" && message.encoding != "bgr8")
    throw std::runtime_error("Color encoding must be rgb8 or bgr8");
  cv::Mat image = copy_pixels(message, CV_8UC3);
  if (message.encoding == "bgr8") cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
  return image;
}

inline cv::Mat depth_metres(const sensor_msgs::msg::Image &message,
                            bool allow_integer, double integer_scale) {
  if (message.encoding == "32FC1") return copy_pixels(message, CV_32FC1);
  if (message.encoding != "16UC1" || !allow_integer)
    throw std::runtime_error("Depth encoding is not supported by this input profile");
  if (!std::isfinite(integer_scale) || integer_scale <= 0)
    throw std::runtime_error("Integer depth scale must be finite and positive");
  cv::Mat metres;
  copy_pixels(message, CV_16UC1).convertTo(metres, CV_32FC1, integer_scale);
  return metres;
}
}  // namespace x3_rgbd
