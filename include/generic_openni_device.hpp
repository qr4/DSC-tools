#pragma once

// C++ libs
#include <memory>
#include <mutex>
#include <unordered_map>
#include <thread>
#include <tuple>
#include <condition_variable>

// Boost
#include <boost/property_tree/ptree.hpp>

// OpenCV
#include <opencv2/opencv.hpp>
// OpenNI
#include <OpenNI.h>

// Project Includes
#include <utility_functions.hpp>
#include <enums.hpp>

struct SensorConfig {
  bool mirror;
  // this contains the resolution, fps and the pixel format
  openni::VideoMode video_mode;
  bool compute_grayscale_image;

  // only used by the depth sensor
  bool compute_point3f_image;
};

class GenericOpenNIDevice
{
  public:
    GenericOpenNIDevice();
    ~GenericOpenNIDevice();

    void init(const boost::property_tree::ptree &ptree);

    // open the device with an uri
    bool open();
    bool close();

    std::string getSerial() const;

    std::pair<float, float> getColorFov() const;
    std::pair<float, float> getDepthFov() const;

    std::unordered_map<ImageType, std::tuple<uint64_t, cv::Mat>> getImages();

    static std::string listAllDevices();

  private:
    static bool initializeOpenNI();
    static void fetchDeviceInformation();
    openni::VideoMode getVideoModeFromConfig(const boost::property_tree::ptree &ptree);

    bool createStream(const openni::SensorType &type);
    void capturingThread();
    void getNextImages();

    cv::Mat transformRGBFrameToMat(const openni::VideoFrameRef &frame);

    cv::Mat transformDepthFrameToMillimeterMat(const openni::VideoFrameRef &frame);
    cv::Mat convertMillimeterMatToRGB(const cv::Mat &image);

    cv::Mat transformIRFrameToMat(const openni::VideoFrameRef &frame);
    cv::Mat convertIRMatToRGB(const cv::Mat &image);

    cv::Mat alignColorMatToDepthMat(const cv::Mat &color_image, const cv::Mat &depth_image);
    cv::Mat convertMillimeterMatToPoint3fMat(const cv::Mat &depth_mat);

  private:
    std::unordered_map<openni::SensorType, SensorConfig> streams_configs_;

    std::string serial_;
    std::string uri_;
    openni::Device device_;
    std::unordered_map<openni::SensorType, std::unique_ptr<openni::VideoStream>> streams_;

    std::thread capturing_thread_;
    std::condition_variable thread_set_up_condition_;

    std::condition_variable new_images_condition_;
    std::mutex capturing_mutex_;

    bool newImagesAvailable_;
    std::unordered_map<ImageType, std::tuple<uint64_t, cv::Mat>> images_;

    static std::unordered_map<std::string, openni::DeviceInfo> device_information_map_;
};