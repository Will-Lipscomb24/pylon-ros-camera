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

TEST(ImageCompression, CompressesAndResizesLargeColorImageLoadedFromDiskToJpeg)
{
  constexpr int kSourceWidth = 4096;
  constexpr int kSourceHeight = 3000;
  constexpr int kTargetWidth = 1536;
  constexpr int kTargetHeight = 1125;
  const ScopedTempFile temp_file(makeTempPath("_resized_color.png"));

  const cv::Mat generated = makeLargeColorPattern(kSourceWidth, kSourceHeight);
  ASSERT_TRUE(cv::imwrite(temp_file.path(), generated));

  const cv::Mat image_from_disk = cv::imread(temp_file.path(), cv::IMREAD_COLOR);
  ASSERT_FALSE(image_from_disk.empty());
  ASSERT_EQ(image_from_disk.cols, kSourceWidth);
  ASSERT_EQ(image_from_disk.rows, kSourceHeight);

  const auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(),
                                            sensor_msgs::image_encodings::BGR8,
                                            image_from_disk).toImageMsg();

  ImageCompressionOptions options;
  options.format = "jpeg";
  options.jpeg_quality = 80;
  options.target_width = kTargetWidth;
  options.target_height = kTargetHeight;

  sensor_msgs::msg::CompressedImage compressed_msg;
  std::string error_message;
  ASSERT_TRUE(compressImageMessage(*image_msg, options, compressed_msg, error_message)) << error_message;
  EXPECT_EQ(compressed_msg.format, "bgr8; jpeg compressed bgr8");
  EXPECT_LT(compressed_msg.data.size(), image_msg->data.size());

  const cv::Mat decoded = cv::imdecode(compressed_msg.data, cv::IMREAD_COLOR);
  ASSERT_FALSE(decoded.empty());
  EXPECT_EQ(decoded.cols, kTargetWidth);
  EXPECT_EQ(decoded.rows, kTargetHeight);
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

TEST(ImageCompression, RejectsResizeWhenOnlyOneTargetDimensionProvided)
{
  ImageCompressionOptions options;
  options.target_width = 1024;

  std::string error_message;
  EXPECT_FALSE(validateResizeTarget(options, error_message));
  EXPECT_NE(error_message.find("both target_width and target_height"), std::string::npos);
}

TEST(ImageCompression, RejectsResizeTargetLargerThanSource)
{
  ImageCompressionOptions options;
  options.target_width = 4097;
  options.target_height = 3000;

  std::string error_message;
  EXPECT_FALSE(validateResizeTargetAgainstSource(options, 4096, 3000, error_message));
  EXPECT_NE(error_message.find("smaller than or equal"), std::string::npos);
}

TEST(ImageCompression, RejectsResizeAspectRatioMismatch)
{
  ImageCompressionOptions options;
  options.target_width = 2048;
  options.target_height = 1200;

  std::string error_message;
  EXPECT_FALSE(validateResizeTargetAgainstSource(options, 4096, 3000, error_message));
  EXPECT_NE(error_message.find("preserve the source aspect ratio"), std::string::npos);
}

TEST(ImageCompression, SelectsLargestEqualHardwareBinningCandidate)
{
  EXPECT_EQ(selectPreferredEqualBinningFactor(4096, 3000, 1536, 1125), 2u);
  EXPECT_EQ(selectPreferredEqualBinningFactor(4096, 3000, 2048, 1500), 2u);
  EXPECT_EQ(selectPreferredEqualBinningFactor(4096, 3000, 4096, 3000), 1u);
}

}  // namespace pylon_ros2_camera
