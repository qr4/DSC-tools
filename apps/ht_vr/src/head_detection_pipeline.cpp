#include "head_detection_pipeline.hpp"
#include <numeric>

void HeadDetectionPipeline::init(const std::string &classifier_config) {
  classifier_ = cv::CascadeClassifier(classifier_config);
}

void HeadDetectionPipeline::setImages(const std::unordered_map<ImageType, std::tuple<uint64_t, cv::Mat>> &images) {
  images_ = images;
}

void HeadDetectionPipeline::setVisualizer(const std::shared_ptr<Visualizer> &vis) {
  visualizer_ = vis;
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

  if (!faces.size()) return;

  std::vector<cv::Point3f> face_points;

  auto face = faces.at(0);
  for (int y = face.y; y < face.y + face.width; ++y) {
    for (int x = face.x; x < face.x + face.height; ++x) {
      auto p3f_pt = p3f_image.at<cv::Point3f>(y, x);
      if (p3f_pt.x == 0 && p3f_pt.y == 0 &&  p3f_pt.z == 0) continue;
      // FIXME
      if (p3f_pt.z < -1.5) continue;
      face_points.push_back(p3f_pt);
    }
  }

  float size = face_points.size();

  cv::Point3f acc = std::accumulate(face_points.begin(), face_points.end(), cv::Point3f(0,0,0),
      [&] (cv::Point3f &p, cv::Point3f &acc) {
        return cv::Point3f(p.x + acc.x, p.y + acc.y, p.z + acc.z);
      });

  acc.x /= size;
  acc.y /= size;
  acc.z /= size;

  computed_head_pos_ = acc;
  // TODO add tracking
}

cv::Point3f HeadDetectionPipeline::getHeadPos() const {
  return computed_head_pos_;
}

