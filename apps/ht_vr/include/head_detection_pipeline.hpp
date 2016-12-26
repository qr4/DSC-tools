#pragma once

// STL
#include <unordered_map>
#include <tuple>

// Boost
#include <boost/property_tree/ptree.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// Project Includes
#include "enums.hpp"


class HeadDetectionPipeline {

  public:
    HeadDetectionPipeline() = default;
    ~HeadDetectionPipeline() = default;

    void init(const std::string &classifier_config);

    void setImages(const std::unordered_map<ImageType, std::tuple<uint64_t, cv::Mat>> &images);

    void compute();

    cv::Point3f getHeadPos() const;

  private:

    cv::CascadeClassifier classifier_;

    // input
    std::unordered_map<ImageType, std::tuple<uint64_t, cv::Mat>> images_;

    // output
    cv::Point3f computed_head_pos_;

};
