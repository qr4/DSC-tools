#include "head_detection_pipeline.hpp"

void HeadDetectionPipeline::init(const std::string &classifier_config) {
  classifier_ = cv::CascadeClassifier(classifier_config);
}

void HeadDetectionPipeline::setImages(const std::unordered_map<ImageType, std::tuple<uint64_t, cv::Mat>> &images) {
  images_ = images;
}

void HeadDetectionPipeline::compute() {
  // first, detect a face in the color image
  const auto color_image = std::get<1>(images_.at(ImageType::RGB));
  const auto p3f_image = std::get<1>(images_.at(ImageType::POINT_3F));

  cv::Mat gray_image;
  cv::cvtColor(color_image, gray_image, cv::COLOR_RGB2GRAY);

  std::vector<cv::Rect> faces;
  classifier_.detectMultiScale(gray_image, faces, 1.3, 5);
  std::cout << "found " << faces.size() << " faces" << std::endl;;

  // next, determine the corresponding position in 3d
  // TODO add tracking
}

cv::Point3f HeadDetectionPipeline::getHeadPos() const {
  return computed_head_pos_;
}

