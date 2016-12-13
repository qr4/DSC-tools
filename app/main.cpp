// C++ libs
#include <iostream>
#include <string>
#include <fstream>

// Boost
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// Project includes
#include "generic_openni_device.hpp"

struct Configuration {
  std::string config_file;
};

bool processCommandLine(int argc, char** argv, Configuration &config) {
  boost::program_options::options_description desc("Program options");

  desc.add_options()
    ("help,h", "Print this help message")
    ("list,l", "List all connected devices")
    ("config,c", boost::program_options::value<std::string>(&config.config_file))
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

int main(int argc, char* argv[]) {

  Configuration config;
  if (!processCommandLine(argc, argv, config)) return 1;

  boost::filesystem::path configuration_file(boost::filesystem::absolute(config.config_file));
  boost::property_tree::ptree ptree;
  boost::property_tree::read_json(configuration_file.string(), ptree);

  auto device = new GenericOpenNIDevice();

  device->init(ptree);
  if (!device->open()) return 1;

  cv::namedWindow("color", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("depth-gray", cv::WINDOW_AUTOSIZE);

  while (true) {
    auto images = device->getImages();
    auto depthMmImage = std::get<1>(images.at(ImageType::DEPTH_MM));
    auto depthGrayImage = std::get<1>(images.at(ImageType::DEPTH_GRAYSCALE));
    auto rgbImage = std::get<1>(images.at(ImageType::RGB_ALIGNED));

    cv::imshow("color", rgbImage);
    cv::imshow("depth-gray", depthGrayImage);
    auto key = cv::waitKey(1);
    if (key == 27 /* Esc */) break;

  }
}