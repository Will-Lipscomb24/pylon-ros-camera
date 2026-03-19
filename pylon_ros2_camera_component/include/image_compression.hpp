/****************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2026. All rights reserved.
 *****************************************************************************/

#pragma once

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
};

bool compressImageMessage(const sensor_msgs::msg::Image& image_msg,
                          const ImageCompressionOptions& options,
                          sensor_msgs::msg::CompressedImage& compressed_msg,
                          std::string& error_message);

}  // namespace pylon_ros2_camera
