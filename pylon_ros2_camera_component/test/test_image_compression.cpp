/****************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2026. All rights reserved.
 *****************************************************************************/

#include "image_compression.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>

namespace pylon_ros2_camera
{
namespace
{
class ScopedTempFile
{
public:
  explicit ScopedTempFile(std::string path)
  : path_(std::move(path))
  {
  }

  ~ScopedTempFile()
  {
    if (!path_.empty())
    {
      std::remove(path_.c_str());
    }
  }

  const std::string& path() const
  {
    return path_;
  }

private:
  std::string path_;
};

std::string makeTempPath(const std::string& suffix)
{
  return "/tmp/pylon_ros2_camera_image_compression_" + std::to_string(::getpid()) + suffix;
}

cv::Mat makeLargeColorPattern(const int width, const int height)
{
  cv::Mat image(height, width, CV_8UC3);
  for (int y = 0; y < height; ++y)
  {
    cv::Vec3b* row = image.ptr<cv::Vec3b>(y);
    for (int x = 0; x < width; ++x)
    {
      row[x][0] = static_cast<unsigned char>(x % 256);
      row[x][1] = static_cast<unsigned char>(y % 256);
      row[x][2] = static_cast<unsigned char>((x / 4 + y / 4) % 256);
    }
  }
  return image;
}

cv::Mat makeLargeMono16Pattern(const int width, const int height)
{
  cv::Mat image(height, width, CV_16UC1);
  for (int y = 0; y < height; ++y)
  {
    std::uint16_t* row = image.ptr<std::uint16_t>(y);
    for (int x = 0; x < width; ++x)
    {
      row[x] = static_cast<std::uint16_t>((x * 17 + y * 31) % 65535);
    }
  }
  return image;
}
}  // namespace

TEST(ImageCompression, CompressesLargeColorImageLoadedFromDiskToJpeg)
{
  constexpr int kWidth = 4096;
  constexpr int kHeight = 3000;
  const ScopedTempFile temp_file(makeTempPath("_color.png"));

  const cv::Mat generated = makeLargeColorPattern(kWidth, kHeight);
  ASSERT_TRUE(cv::imwrite(temp_file.path(), generated));

  const cv::Mat image_from_disk = cv::imread(temp_file.path(), cv::IMREAD_COLOR);
  ASSERT_FALSE(image_from_disk.empty());
  ASSERT_EQ(image_from_disk.cols, kWidth);
  ASSERT_EQ(image_from_disk.rows, kHeight);

  const auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(),
                                            sensor_msgs::image_encodings::BGR8,
                                            image_from_disk).toImageMsg();

  ImageCompressionOptions options;
  options.format = "jpeg";
  options.jpeg_quality = 80;

  sensor_msgs::msg::CompressedImage compressed_msg;
  std::string error_message;
  ASSERT_TRUE(compressImageMessage(*image_msg, options, compressed_msg, error_message)) << error_message;
  EXPECT_EQ(compressed_msg.format, "bgr8; jpeg compressed bgr8");
  EXPECT_LT(compressed_msg.data.size(), image_msg->data.size());

  const cv::Mat decoded = cv::imdecode(compressed_msg.data, cv::IMREAD_COLOR);
  ASSERT_FALSE(decoded.empty());
  EXPECT_EQ(decoded.cols, kWidth);
  EXPECT_EQ(decoded.rows, kHeight);
}

TEST(ImageCompression, CompressesLargeMono16ImageLoadedFromDiskToPng)
{
  constexpr int kWidth = 4096;
  constexpr int kHeight = 3000;
  const ScopedTempFile temp_file(makeTempPath("_mono16.png"));

  const cv::Mat generated = makeLargeMono16Pattern(kWidth, kHeight);
  ASSERT_TRUE(cv::imwrite(temp_file.path(), generated));

  const cv::Mat image_from_disk = cv::imread(temp_file.path(), cv::IMREAD_UNCHANGED);
  ASSERT_FALSE(image_from_disk.empty());
  ASSERT_EQ(image_from_disk.type(), CV_16UC1);
  ASSERT_EQ(image_from_disk.cols, kWidth);
  ASSERT_EQ(image_from_disk.rows, kHeight);

  const auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(),
                                            sensor_msgs::image_encodings::MONO16,
                                            image_from_disk).toImageMsg();

  ImageCompressionOptions options;
  options.format = "png";
  options.png_level = 3;

  sensor_msgs::msg::CompressedImage compressed_msg;
  std::string error_message;
  ASSERT_TRUE(compressImageMessage(*image_msg, options, compressed_msg, error_message)) << error_message;
  EXPECT_EQ(compressed_msg.format, "mono16; png compressed");
  EXPECT_LT(compressed_msg.data.size(), image_msg->data.size());

  const cv::Mat decoded = cv::imdecode(compressed_msg.data, cv::IMREAD_UNCHANGED);
  ASSERT_FALSE(decoded.empty());
  EXPECT_EQ(decoded.type(), CV_16UC1);
  EXPECT_EQ(decoded.cols, kWidth);
  EXPECT_EQ(decoded.rows, kHeight);
}

}  // namespace pylon_ros2_camera
