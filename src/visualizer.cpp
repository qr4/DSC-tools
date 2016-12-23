#include "visualizer.hpp"
#include <thread>
#include <memory>


Visualizer::Visualizer() {
  pcl_visualizer_.reset(new pcl::visualization::PCLVisualizer());
  pcl_visualizer_->getRenderWindow()->MakeCurrent();
  pcl_visualizer_->addCoordinateSystem(1.0);
  pcl_visualizer_->setCameraClipDistances(0.1, 10);

  shutdown_ = false;

  thread_ = std::thread(std::bind(&Visualizer::threadFunc, this));
}

Visualizer::~Visualizer() {
  shutdown_ = true;
  thread_.join();
}

void Visualizer::threadFunc() {
  std::unique_lock<std::mutex> lk(mutex_, std::defer_lock);
  while(!shutdown_) {

    lk.lock();
    pcl_visualizer_->getRenderWindow()->MakeCurrent();
    pcl_visualizer_->spinOnce(1, true);
    lk.unlock();

    std::this_thread::sleep_for(std::chrono::milliseconds(33));
  }
}
