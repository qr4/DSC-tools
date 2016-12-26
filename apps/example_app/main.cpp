// C++ libs
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

// Boost
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// Project includes
#include "generic_openni_device.hpp"
#include "utility_functions.hpp"
#include "visualizer.hpp"

struct Configuration {
  // parsed from command line
  std::string config_file;
  std::string record_path;
  std::string input_path;

  // computed
  bool record;
  bool with_recording;
};

bool processCommandLine(int argc, char** argv, Configuration &config) {
  boost::program_options::options_description desc("Program options");

  desc.add_options()
    ("help,h", "Print this help message")
    ("list,l", "List all connected devices")
    ("config,c", boost::program_options::value<std::string>(&config.config_file))
    ("record,r", boost::program_options::value<std::string>(&config.record_path))
    ("input,i", boost::program_options::value<std::string>(&config.input_path))
    ;
  try
  {
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return false;
    }

    if (vm.count("list")) {
      std::cout << GenericOpenNIDevice::listAllDevices() << std::endl;
      return false;
    }

    if (!vm.count("config")) {
      std::cout << "You must provide a config file." << std::endl << desc << std::endl;
      return false;
    }
    config.record = vm.count("record");
    config.with_recording = vm.count("input");

    if (config.record && config.with_recording) {
      std::cout << "Input and record are mutually exclusive."<< std::endl << desc << std::endl;
      return false;
    }

    boost::program_options::notify(vm);
  } catch (boost::program_options::required_option &e) {
    std::cerr << "Error! " << e.what() << std::endl;
    return false;
  } catch (...) {
    std::cerr << "Unknown error while parsing the command line!" << std::endl;
    return false;
  }
  return true;
}

// FIXME: ugly globals
std::atomic<bool> paused(false);
std::atomic<bool> shutdown(false);

void keyboardCallback(const pcl::visualization::KeyboardEvent &ev) {
  std::cout << "key was pressed: " << ev.getKeySym() << std::endl;
  if (ev.keyDown()) {
    if (ev.getKeySym() == "space") {
      paused = !paused;
    } else if (ev.getKeySym() == "Escape" || ev.getKeySym() == "q") {
      shutdown = true;
    }
  }
}

int main(int argc, char* argv[]) {

  Configuration config;
  if (!processCommandLine(argc, argv, config)) return 1;

  boost::filesystem::path configuration_file(boost::filesystem::absolute(config.config_file));
  boost::property_tree::ptree ptree;
  boost::property_tree::read_json(configuration_file.string(), ptree);

  Visualizer vis;
  vis.registerKeyboardCallback(keyboardCallback);

  auto device = new GenericOpenNIDevice();

  device->init(ptree);

  if (config.with_recording) {
    if (!device->openWithUri(config.input_path)) return 1;
  } else {
    if (!device->open()) return 1;
  }

  if (config.record) {
    device->startRecording(config.record_path);
  }

  vis.addPointCloud<pcl::PointXYZ>(
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()), "cam0");

  while (!shutdown) {
    if (paused) continue;

    auto images = device->getImages();

    auto rgb_image = std::get<1>(images.at(ImageType::RGB_ALIGNED));
    auto p3f_image = std::get<1>(images.at(ImageType::POINT_3F));

    auto point_cloud = getPointCloud(p3f_image, rgb_image);
    vis.updatePointCloud<pcl::PointXYZRGB>(point_cloud, "cam0");
  }

  device->close();
}
