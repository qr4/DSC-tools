#pragma once

#include <string>
#include <OpenNI.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <enums.hpp>

std::string imageTypeToString(const ImageType &type) {
  switch (type) {
    case ImageType::DEPTH_MM:
      return "Depth Millimeter-ImageType";
    case ImageType::DEPTH_GRAYSCALE:
      return "Depth Grayscale ImageType";
    case ImageType::POINT_3F:
      return "Point3f ImageType";
    case ImageType::RGB:
      return "RGB ImageType";
    case ImageType::IR:
        return "IR ImageType";
    case ImageType::IR_GRAYSCALE:
        return "IR Grayscale ImageType";
    default:
      return "Unknown Type";
  };
}

std::string openNISensorTypeToString(const openni::SensorType &type) {
  switch (type) {
    case openni::SENSOR_IR: return "SENSOR_IR";
    case openni::SENSOR_COLOR: return "SENSOR_COLOR";
    case openni::SENSOR_DEPTH: return "SENSOR_DEPTH";
    default: return "Unknown sensor type";
  };
}

openni::PixelFormat stringToOpenNIPixelFormat(const std::string &str) {
  if (str == "PIXEL_FORMAT_DEPTH_1_MM") return openni::PIXEL_FORMAT_DEPTH_1_MM;
  if (str == "PIXEL_FORMAT_DEPTH_100_UM") return openni::PIXEL_FORMAT_DEPTH_100_UM;
  if (str == "PIXEL_FORMAT_SHIFT_9_2") return openni::PIXEL_FORMAT_SHIFT_9_2;
  if (str == "PIXEL_FORMAT_SHIFT_9_3") return openni::PIXEL_FORMAT_SHIFT_9_3;
  if (str == "PIXEL_FORMAT_RGB888") return openni::PIXEL_FORMAT_RGB888;
  if (str == "PIXEL_FORMAT_YUV422") return openni::PIXEL_FORMAT_YUV422;
  if (str == "PIXEL_FORMAT_GRAY8") return openni::PIXEL_FORMAT_GRAY8;
  if (str == "PIXEL_FORMAT_GRAY16") return openni::PIXEL_FORMAT_GRAY16;
  if (str == "PIXEL_FORMAT_JPEG") return openni::PIXEL_FORMAT_JPEG;
  if (str == "PIXEL_FORMAT_YUYV") return openni::PIXEL_FORMAT_YUYV;

  throw std::runtime_error("invalid string for openni pixel format: " + str);
}

std::string openNIPixelFormatToString(const openni::PixelFormat &format) {
  switch (format) {
    case openni::PIXEL_FORMAT_DEPTH_1_MM: return "PIXEL_FORMAT_DEPTH_1_MM";
    case openni::PIXEL_FORMAT_DEPTH_100_UM: return "PIXEL_FORMAT_DEPTH_100_UM";
    case openni::PIXEL_FORMAT_SHIFT_9_2: return "PIXEL_FORMAT_SHIFT_9_2";
    case openni::PIXEL_FORMAT_SHIFT_9_3: return "PIXEL_FORMAT_SHIFT_9_3";

    case openni::PIXEL_FORMAT_RGB888: return "PIXEL_FORMAT_RGB888";
    case openni::PIXEL_FORMAT_YUV422: return "PIXEL_FORMAT_YUV422";
    case openni::PIXEL_FORMAT_GRAY8: return "PIXEL_FORMAT_GRAY8";
    case openni::PIXEL_FORMAT_GRAY16: return "PIXEL_FORMAT_GRAY16";
    case openni::PIXEL_FORMAT_JPEG: return "PIXEL_FORMAT_JPEG";
    case openni::PIXEL_FORMAT_YUYV: return "PIXEL_FORMAT_YUYV";
    default: return "Unknown format";
  };
}

// Image to PointCloud transformations
pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud(const cv::Mat_<cv::Point3f> &p3f_image) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (int y = 0; y < p3f_image.rows; ++y) {
    const cv::Point3f *row = p3f_image.ptr<cv::Point3f>(y);

    for (int x = 0; x < p3f_image.cols; ++x) {
      pcl::PointXYZ pt(row[x].x, row[x].y, row[x].z);
      cloud->push_back(pt);
    }
  }
  return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud(const cv::Mat_<cv::Point3f> &p3f_image,
                                                  const cv::Mat_<cv::Vec3b> &color_image) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  for (int y = 0; y < p3f_image.rows; ++y) {
    const cv::Point3f *depth_row = p3f_image.ptr<cv::Point3f>(y);
    const cv::Vec3b *color_row = p3f_image.ptr<cv::Vec3b>(y);

    for (int x = 0; x < p3f_image.cols; ++x) {
      pcl::PointXYZRGB pt(depth_row[x].x, depth_row[x].y, depth_row[x].z);
      pt.r = color_row[x][0];
      pt.r = color_row[x][1];
      pt.r = color_row[x][2];

      cloud->push_back(pt);
    }
  }
  return cloud;
}

pcl::PointCloud<pcl::PointXZYRGB>::Ptr getPointCloud(const cv::Mat_<cv::Point3f> &p3f_image, cv::Scalar &color) {
  cv::Mat color_image(p3f_image.size(), CV_8UC3, color);
  return getPointCloud(p3f_image, color_image);
}

