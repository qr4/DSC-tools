#pragma once

// STL
#include <string>
#include <thread>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <atomic>

// PCL
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>

// Boost
#include <boost/function.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// This class is a threaded wrapper around PCLVisualizer, that is: The visualization runs
// in its own thread. If want to manipuliate some part of the state of the underlying visualizer,
// you will acquire a lock.

class Visualizer {
  public:
    Visualizer();
    ~Visualizer();

    template<typename PointT>
      void addSphere(const PointT &pt, const double radius,
                     const double r = 1., const double g = 1., const double b = 1.,
                     const std::string &id = "sphere", const int viewport = 0) {
        callFuncSync([&](){pcl_visualizer_->addSphere(pt, radius, r, g, b, id, viewport);});
      }

    template<typename PointT>
      void addPointCloud(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                         const std::string &id = "cloud",
                         const int viewport = 0) {
        callFuncSync([&](){pcl_visualizer_->addPointCloud(cloud, id, viewport);});
      }

    template<typename PointT>
      void updatePointCloud(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                            const std::string &id = "cloud") {
        callFuncSync([&](){pcl_visualizer_->updatePointCloud(cloud, id);});
      }

    void removePointCloud(const std::string &id = "cloud", const int viewport = 0) {
      callFuncSync([&](){pcl_visualizer_->removePointCloud(id, viewport);});
    }

    void registerKeyboardCallback(
        boost::function<void (const pcl::visualization::KeyboardEvent&)> cb) {
      callFuncSync([&](){pcl_visualizer_->registerKeyboardCallback(cb);});
    }

  private:
    Visualizer(const Visualizer&) = delete;
    Visualizer(Visualizer&&) = delete;
    Visualizer& operator=(const Visualizer&) = delete;

    void threadFunc();

    // synchronization stuff:
    template<typename Fn>
    void callFuncSync(const Fn &fn) {
      std::lock_guard<std::mutex> lk(mutex_);
      pcl_visualizer_->getRenderWindow()->MakeCurrent();
      fn();
    }

    std::thread thread_;
    std::mutex mutex_;

    std::atomic<bool> shutdown_;

    // the underlying pcl visualizer
    pcl::visualization::PCLVisualizer::Ptr pcl_visualizer_;
};
