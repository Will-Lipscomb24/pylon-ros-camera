/****************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2026. All rights reserved.
 *****************************************************************************/

#pragma once

#include <cstddef>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <string>

namespace pylon_ros2_camera
{

struct ImageCompressionOptions
{
  std::string format{"jpeg"};
  int jpeg_quality{80};
  int png_level{3};
  int target_width{0};
  int target_height{0};
};

bool hasResizeTarget(const ImageCompressionOptions& options);

bool validateResizeTarget(const ImageCompressionOptions& options,
                          std::string& error_message);

bool validateResizeTargetAgainstSource(const ImageCompressionOptions& options,
                                       std::size_t source_width,
                                       std::size_t source_height,
                                       std::string& error_message);

std::size_t selectPreferredEqualBinningFactor(std::size_t source_width,
                                              std::size_t source_height,
                                              std::size_t target_width,
                                              std::size_t target_height);

bool compressImageMessage(const sensor_msgs::msg::Image& image_msg,
                          const ImageCompressionOptions& options,
                          sensor_msgs::msg::CompressedImage& compressed_msg,
                          std::string& error_message);

}  // namespace pylon_ros2_camera
