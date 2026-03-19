/****************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2026. All rights reserved.
 *****************************************************************************/

#include "image_compression.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace pylon_ros2_camera
{
namespace
{
std::string normalizeFormat(const std::string& format)
{
  std::string normalized = format;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (normalized == "jpg")
  {
    return "jpeg";
  }

  return normalized;
}

bool isColorLikeEncoding(const std::string& encoding)
{
  return sensor_msgs::image_encodings::isColor(encoding) ||
         sensor_msgs::image_encodings::isBayer(encoding) ||
         encoding == sensor_msgs::image_encodings::YUV422 ||
         encoding == sensor_msgs::image_encodings::YUV422_YUY2;
}

bool buildCompressionPlan(const sensor_msgs::msg::Image& image_msg,
                          const std::string& normalized_format,
                          std::string& cv_bridge_encoding,
                          std::string& compressed_format,
                          std::string& error_message)
{
  const std::string& original_encoding = image_msg.encoding;
  const bool is_mono = sensor_msgs::image_encodings::isMono(original_encoding);
  const bool is_color_like = isColorLikeEncoding(original_encoding);

  if (!is_mono && !is_color_like)
  {
    error_message = "Unsupported image encoding for compression: '" + original_encoding + "'";
    return false;
  }

  const int bit_depth = sensor_msgs::image_encodings::bitDepth(original_encoding);
  compressed_format = original_encoding + "; " + normalized_format + " compressed";

  if (normalized_format == "jpeg")
  {
    if (bit_depth != 8)
    {
      error_message = "JPEG compression only supports 8-bit image encodings";
      return false;
    }

    if (is_mono)
    {
      cv_bridge_encoding = sensor_msgs::image_encodings::MONO8;
      return true;
    }

    cv_bridge_encoding = sensor_msgs::image_encodings::BGR8;
    compressed_format += " " + cv_bridge_encoding;
    return true;
  }

  if (normalized_format == "png")
  {
    if (bit_depth != 8 && bit_depth != 16)
    {
      error_message = "PNG compression only supports 8-bit and 16-bit image encodings";
      return false;
    }

    if (is_mono)
    {
      cv_bridge_encoding = (bit_depth == 16)
                               ? sensor_msgs::image_encodings::MONO16
                               : sensor_msgs::image_encodings::MONO8;
      return true;
    }

    cv_bridge_encoding = (bit_depth == 16)
                             ? sensor_msgs::image_encodings::BGR16
                             : sensor_msgs::image_encodings::BGR8;
    compressed_format += " " + cv_bridge_encoding;
    return true;
  }

  error_message = "Unsupported compressed image format: '" + normalized_format + "'";
  return false;
}
}  // namespace

bool hasResizeTarget(const ImageCompressionOptions& options)
{
  return options.target_width > 0 || options.target_height > 0;
}

bool validateResizeTarget(const ImageCompressionOptions& options,
                          std::string& error_message)
{
  error_message.clear();

  if (options.target_width < 0 || options.target_height < 0)
  {
    error_message = "compressed image resize target dimensions must be greater than or equal to zero";
    return false;
  }

  if (!hasResizeTarget(options))
  {
    return true;
  }

  if (options.target_width <= 0 || options.target_height <= 0)
  {
    error_message = "compressed image resize requires both target_width and target_height";
    return false;
  }

  return true;
}

bool validateResizeTargetAgainstSource(const ImageCompressionOptions& options,
                                       const std::size_t source_width,
                                       const std::size_t source_height,
                                       std::string& error_message)
{
  if (!validateResizeTarget(options, error_message))
  {
    return false;
  }

  if (!hasResizeTarget(options))
  {
    return true;
  }

  if (source_width == 0 || source_height == 0)
  {
    error_message = "source image geometry is invalid";
    return false;
  }

  const std::size_t target_width = static_cast<std::size_t>(options.target_width);
  const std::size_t target_height = static_cast<std::size_t>(options.target_height);

  if (target_width > source_width || target_height > source_height)
  {
    error_message = "compressed image resize target must be smaller than or equal to the source image";
    return false;
  }

  if (source_width * target_height != source_height * target_width)
  {
    error_message = "compressed image resize target must preserve the source aspect ratio";
    return false;
  }

  return true;
}

std::size_t selectPreferredEqualBinningFactor(const std::size_t source_width,
                                              const std::size_t source_height,
                                              const std::size_t target_width,
                                              const std::size_t target_height)
{
  if (source_width == 0 || source_height == 0 || target_width == 0 || target_height == 0)
  {
    return 1;
  }

  return std::max<std::size_t>(1, std::min(source_width / target_width, source_height / target_height));
}

bool compressImageMessage(const sensor_msgs::msg::Image& image_msg,
                          const ImageCompressionOptions& options,
                          sensor_msgs::msg::CompressedImage& compressed_msg,
                          std::string& error_message)
{
  error_message.clear();

  const std::string normalized_format = normalizeFormat(options.format);
  std::string cv_bridge_encoding;
  std::string compressed_format;
  if (!buildCompressionPlan(image_msg,
                            normalized_format,
                            cv_bridge_encoding,
                            compressed_format,
                            error_message))
  {
    return false;
  }

  cv_bridge::CvImagePtr cv_image;
  try
  {
    cv_image = cv_bridge::toCvCopy(image_msg, cv_bridge_encoding);
  }
  catch (const cv_bridge::Exception& ex)
  {
    error_message = std::string("cv_bridge conversion failed: ") + ex.what();
    return false;
  }

  std::vector<int> encode_params;
  std::string extension;
  if (normalized_format == "jpeg")
  {
    extension = ".jpg";
    encode_params = {cv::IMWRITE_JPEG_QUALITY, options.jpeg_quality};
  }
  else
  {
    extension = ".png";
    encode_params = {cv::IMWRITE_PNG_COMPRESSION, options.png_level};
  }

  if (!validateResizeTargetAgainstSource(options,
                                         image_msg.width,
                                         image_msg.height,
                                         error_message))
  {
    return false;
  }

  cv::Mat image_to_encode = cv_image->image;
  cv::Mat resized_image;
  if (hasResizeTarget(options) &&
      (image_msg.width != static_cast<std::uint32_t>(options.target_width) ||
       image_msg.height != static_cast<std::uint32_t>(options.target_height)))
  {
    cv::resize(cv_image->image,
               resized_image,
               cv::Size(options.target_width, options.target_height),
               0.0,
               0.0,
               cv::INTER_AREA);
    image_to_encode = resized_image;
  }

  compressed_msg.header = image_msg.header;
  compressed_msg.format = compressed_format;
  if (!cv::imencode(extension, image_to_encode, compressed_msg.data, encode_params))
  {
    error_message = "OpenCV failed to encode compressed image";
    return false;
  }

  return true;
}

}  // namespace pylon_ros2_camera
