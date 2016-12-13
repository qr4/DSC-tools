#include <generic_openni_device.hpp>
#include <unordered_map>
#include <iostream>

#include <OpenNI.h>

// static member initialization
std::unordered_map<std::string, openni::DeviceInfo> GenericOpenNIDevice::device_information_map_;

GenericOpenNIDevice::GenericOpenNIDevice() {}

GenericOpenNIDevice::~GenericOpenNIDevice() {
  close();
}

void GenericOpenNIDevice::init(const boost::property_tree::ptree &ptree) {
  fetchDeviceInformation();

  serial_ = ptree.get<std::string>("serial");
  std::cout << "serial is: " << serial_ << std::endl;
  uri_ = device_information_map_.at(serial_).getUri();
  std::cout << "uri is: " << uri_ << std::endl;
  // TODO set pixel format

  streams_configs_.clear();

  if (ptree.get_child_optional("irStream")) {
    auto stream_ptree = ptree.get_child("irStream");
    streams_configs_[openni::SENSOR_IR] = {
      stream_ptree.get_child_optional("mirror") ? true : false,
      getVideoModeFromConfig(stream_ptree),
      stream_ptree.get_child_optional("computeGrayScaleImage") ? true : false,
      false
    };
  }

  if (ptree.get_child_optional("colorStream")) {
    auto stream_ptree = ptree.get_child("colorStream");
    streams_configs_[openni::SENSOR_COLOR] = {
      stream_ptree.get_child_optional("mirror") ? true : false,
      getVideoModeFromConfig(stream_ptree),
      stream_ptree.get_child_optional("computeGrayScaleImage") ? true : false,
      false
    };
  }

  if (ptree.get_child_optional("depthStream")) {
    auto stream_ptree = ptree.get_child("depthStream");
    streams_configs_[openni::SENSOR_DEPTH] = {
      stream_ptree.get_child_optional("mirror") ? true : false,
      getVideoModeFromConfig(stream_ptree),
      stream_ptree.get_child_optional("computeGrayScaleImage") ? true : false,
      stream_ptree.get_child_optional("computePoint3fImage") ? true : false
    };
  }
}

bool GenericOpenNIDevice::open() {
  if (!initializeOpenNI()) return false;

  std::cout << "Opening GenericOpenNIDevice... " << std::endl;

  auto status = device_.open(uri_.c_str());

  if (status != openni::STATUS_OK) {
    std::cerr << openni::OpenNI::getExtendedError() << std::endl;
    return false;
    openni::OpenNI::shutdown();
    close();
  }

  std::cout << "Opened GenericOpenNIDevice successfully!" << std::endl;
  device_.setDepthColorSyncEnabled(true);

  // create & start the streams
  for (auto& type : streams_configs_) {
    if (device_.hasSensor(type.first) && !createStream(type.first)) {
      return false;
    }
  }

  if (device_.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
    std::cout << "Device supports depth-to-color registration. Using it." << std::endl;
    status = device_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    if (status != openni::STATUS_OK) {
      std::cerr << openni::OpenNI::getExtendedError() << std::endl;
    }
  }

  device_.setDepthColorSyncEnabled(true);

  // check if all streams are valid
  for (const auto& stream : streams_) {
    if (!stream.second || !stream.second->isValid())
    {
      std::cerr << "some streams were invalid" << std::endl;
      return false;
    }
  }

  std::cout << "Waiting for OpenNI images ... " << std::flush;

  // start the capturing thread
  capturing_thread_ = std::thread(std::bind(&GenericOpenNIDevice::capturingThread, this));

  std::unique_lock<std::mutex> lk(capturing_mutex_);
  thread_set_up_condition_.wait(lk); // until capturing thread is set up

  std::cout << "Done!" << std::endl;
  return true;
}

bool GenericOpenNIDevice::close() {
  // TODO terminate capturing thread, and join
  return true;
}

std::string GenericOpenNIDevice::getSerial() const {
  return serial_;
}

std::pair<float, float> GenericOpenNIDevice::getColorFov() const {
  if (!streams_.count(openni::SENSOR_COLOR)) {
     throw std::runtime_error("Tried to get the ColorFOV, but the color stream was not opened!");
  }

  return {streams_.at(openni::SENSOR_COLOR)->getHorizontalFieldOfView(),
          streams_.at(openni::SENSOR_COLOR)->getVerticalFieldOfView()};
}

std::pair<float, float> GenericOpenNIDevice::getDepthFov() const {
  if (!streams_.count(openni::SENSOR_DEPTH)) {
     throw std::runtime_error("Tried to get the DepthFOV, but the depth stream was not opened!");
  }

  return {streams_.at(openni::SENSOR_DEPTH)->getHorizontalFieldOfView(),
          streams_.at(openni::SENSOR_DEPTH)->getVerticalFieldOfView()};

}

std::unordered_map<ImageType, std::tuple<uint64_t, cv::Mat>> GenericOpenNIDevice::getImages() {
  if (!newImagesAvailable_) {
    std::unique_lock<std::mutex> lk(capturing_mutex_);
    new_images_condition_.wait(lk);
  }

  newImagesAvailable_ = false;
  return images_;
}


std::string GenericOpenNIDevice::listAllDevices() {
  if (!initializeOpenNI()) return "";

  GenericOpenNIDevice::fetchDeviceInformation();

  std::stringstream sstream;
  // create string with all device informations
  sstream << "Number of connected Devices " << device_information_map_.size() << std::endl;

  for (const auto& device_entry : device_information_map_) {

    sstream << "Device: " << std::endl;
    sstream << "\tSerial number: " << device_entry.first << std::endl;
    sstream << "\tUri: " << device_entry.second.getUri() << std::endl;
    sstream << "\tName: " << device_entry.second.getName() << std::endl;
  }

  return sstream.str();
}

bool GenericOpenNIDevice::initializeOpenNI() {
  auto status = openni::OpenNI::initialize();
  if (status != openni::STATUS_OK) {
    std::cerr << "Error! Could not initialize OpenNI: " << openni::OpenNI::getExtendedError() << std::endl;
    return false;
  }
  return true;
}


void GenericOpenNIDevice::fetchDeviceInformation() {
  if (!initializeOpenNI()) return;

  // get device information
  openni::Array<openni::DeviceInfo> deviceInfos;
  openni::OpenNI::enumerateDevices(&deviceInfos);

  GenericOpenNIDevice::device_information_map_.clear();

  for (int i = 0; i < deviceInfos.getSize(); ++i) {
    // Open the device to determine the serial...
    auto device = std::make_shared<openni::Device>();
    std::string uri = deviceInfos[i].getUri();
    auto returnCode = device->open(uri.c_str());

    if (returnCode != openni::STATUS_OK) {
      std::cerr << "Error! Could not open device by uri: " <<  uri << std::endl;
      return;
    }

    // read the serial
    char serialNumber[1024];
    returnCode = device->getProperty(ONI_DEVICE_PROPERTY_SERIAL_NUMBER, &serialNumber);

    if (returnCode != openni::STATUS_OK) {
      std::cerr << "Error! Could not get serial number for device uri: " << uri << std::endl;
      return;
    }

    GenericOpenNIDevice::device_information_map_[std::string(serialNumber)] = deviceInfos[i];

    device->close();
  }

}

openni::VideoMode GenericOpenNIDevice::getVideoModeFromConfig(const boost::property_tree::ptree &ptree) {
  // TODO parse more config options here and dont use the defaults?
  openni::VideoMode mode;
  mode.setFps(30);
  mode.setResolution(ptree.get<int>("resolutionX"), ptree.get<int>("resolutionY"));
  mode.setPixelFormat(stringToOpenNIPixelFormat(ptree.get<std::string>("pixelFormat")));

  return mode;
}

bool GenericOpenNIDevice::createStream(const openni::SensorType &type) {

  std::cout << "Creating stream for sensortype: " << openNISensorTypeToString(type) << std::endl;

  auto stream = std::make_unique<openni::VideoStream>();
  auto returnCode = stream->create(device_, type);
  if (returnCode != openni::STATUS_OK) {
    std::cerr << "Error! stream create failed: " << openni::OpenNI::getExtendedError() << std::endl;
    stream->destroy();
    return false;
  }

  const auto *sensorInfo = device_.getSensorInfo(type);
  if (sensorInfo == nullptr) {
    throw std::runtime_error("Tried to get the sensorInfo, but the sensor info was not found!");
  }

  const auto &supported_video_modes = sensorInfo->getSupportedVideoModes();

  std::cout << "The supported video modes are: " << std::endl;

  for (int i = 0; i < supported_video_modes.getSize(); ++i) {
    const auto &mode = supported_video_modes[i];
    std::cout << "VideoMode: \n"
              << "\tResolutionX: " << mode.getResolutionX() << "\n"
              << "\tResolutionY: " << mode.getResolutionY() << "\n"
              << "\tPixelFormat: " << openNIPixelFormatToString(mode.getPixelFormat()) << "\n"
              << "\tFps: " << mode.getFps() << std::endl;
  }

  if (streams_configs_.count(type) == 0) {
    throw std::runtime_error("No stream config for this stream!");
  }

  auto sensor_config = streams_configs_[type];

  if (stream->setVideoMode(sensor_config.video_mode) != openni::STATUS_OK) {
    std::cerr << "Error! setVideoMode failed: " << openni::OpenNI::getExtendedError() << std::endl;
    stream->destroy();
    return false;
  }

  if (stream->start() != openni::STATUS_OK) {
    std::cerr << "Error! stream could not be started: " << openni::OpenNI::getExtendedError() << std::endl;
    stream->destroy();
    return false;
  }

  streams_[type] = std::move(stream);
  return true;
}

void GenericOpenNIDevice::capturingThread() {
  getNextImages();
  thread_set_up_condition_.notify_one();
  while (!device_.isFile()) {
    try {
      getNextImages();
    } catch (std::runtime_error &err) {
      std::cerr << "Error! " << err.what() << std::endl;
      while (true) {
        newImagesAvailable_ = true;
        new_images_condition_.notify_one();
      }
    }
  }
}

void GenericOpenNIDevice::getNextImages()
{
  std::unordered_map<ImageType, std::tuple<uint64_t, cv::Mat>> temp_images;

  for (const auto& stream_pair : streams_) {
    uint64_t timestamp;
    auto&& stream = stream_pair.second;
    if (stream) {
      int changed_index;
      openni::VideoStream* array_stream[] = {stream.get()};
      if (openni::OpenNI::waitForAnyStream(array_stream, 1, &changed_index, 10000) != openni::STATUS_OK) {
        throw std::runtime_error("Wait timeout for stream");
      }
      openni::VideoFrameRef frame;
      stream->readFrame(&frame);
      timestamp = frame.getTimestamp();

      switch (stream->getSensorInfo().getSensorType()) {
        case openni::SENSOR_COLOR:
          {
            cv::Mat image = transformRGBFrameToMat(frame);
            temp_images[ImageType::RGB] = std::make_tuple(timestamp, image);

            break;
          }

        case openni::SENSOR_DEPTH:
          {
            cv::Mat image = transformDepthFrameToMillimeterMat(frame);
            temp_images[ImageType::DEPTH_MM] = std::make_tuple(timestamp, image);

            if (streams_configs_[openni::SENSOR_DEPTH].compute_grayscale_image) {
              temp_images[ImageType::DEPTH_GRAYSCALE] = std::make_tuple(timestamp, convertMillimeterMatToRGB(image));
            }

            if (streams_configs_[openni::SENSOR_DEPTH].compute_point3f_image) {
              temp_images[ImageType::POINT_3F] = std::make_tuple(timestamp, convertMillimeterMatToPoint3fMat(image));
            }
            break;
          }

        case openni::SENSOR_IR:
          {
            cv::Mat image = transformIRFrameToMat(frame);
            temp_images[ImageType::IR] = std::make_tuple(timestamp, image);

            if (streams_configs_[openni::SENSOR_IR].compute_grayscale_image) {
              temp_images[ImageType::IR_GRAYSCALE] = std::make_tuple(timestamp, convertIRMatToRGB(image));
            }

            break;
          }

        default:
          break;
      }
    }
  }

  // special case: if we could get both color and depth image, align the color image
  if (temp_images.count(ImageType::RGB) && temp_images.count(ImageType::DEPTH_MM)) {
    const auto &color_image_tuple = temp_images[ImageType::RGB];
    const auto &depth_image_tuple = temp_images[ImageType::DEPTH_MM];

    cv::Mat image = alignColorMatToDepthMat(std::get<1>(color_image_tuple), std::get<1>(depth_image_tuple));
    temp_images[ImageType::RGB_ALIGNED] = std::make_tuple(std::get<0>(color_image_tuple), image);
  }

  // synchronization stuff
  {
    std::lock_guard<std::mutex> lk(capturing_mutex_);
    images_.swap(temp_images);
    new_images_condition_.notify_one();
  }
}

cv::Mat GenericOpenNIDevice::transformRGBFrameToMat(const openni::VideoFrameRef &frame) {
  const uint8_t* data = reinterpret_cast<const uint8_t*>(frame.getData());
  cv::Mat image = cv::Mat(cv::Size(frame.getWidth(), frame.getHeight()), CV_8UC3);
  memcpy(image.data, data, 3 * image.rows * image.cols);
  return image;
}


cv::Mat GenericOpenNIDevice::transformDepthFrameToMillimeterMat(const openni::VideoFrameRef &frame) {
  const uint16_t* data = reinterpret_cast<const uint16_t*>(frame.getData());
  cv::Mat image = cv::Mat_<uint16_t>(cv::Size(frame.getWidth(), frame.getHeight()));
  memcpy(image.data, data, 2 * image.rows * image.cols);
  return image;
}

cv::Mat GenericOpenNIDevice::convertMillimeterMatToRGB(const cv::Mat &image) {
  cv::Mat res_image;
  float conversion_factor = 255./10000.;

  image.convertTo(res_image, CV_8UC1, conversion_factor);

  cv::cvtColor(res_image, res_image, cv::COLOR_GRAY2RGB);

  return res_image;
}

cv::Mat GenericOpenNIDevice::transformIRFrameToMat(const openni::VideoFrameRef &frame) {
  const uint16_t* data = reinterpret_cast<const uint16_t*>(frame.getData());
  cv::Mat image(cv::Size(frame.getWidth(), frame.getHeight()), CV_8UC1);

  // TODO: which conversion factor to use?
  const float conversion_fac = 1. /10.;

  for (int y = 0; y < image.rows; ++y) {
    uint8_t *row = image.ptr(y);
    for (int x = 0; x < image.cols; ++x) {
      row[x] = *data++ * conversion_fac;
    }
  }

  return image;
}

cv::Mat GenericOpenNIDevice::convertIRMatToRGB(const cv::Mat &image) {
  cv::Mat res_image;
  cv::cvtColor(image, res_image, cv::COLOR_GRAY2RGB);

  return res_image;
}


cv::Mat GenericOpenNIDevice::alignColorMatToDepthMat(const cv::Mat &color_image, const cv::Mat &depth_image) {
  cv::Mat_<cv::Vec3b> res_image(cv::Size(color_image.cols, color_image.rows));
  // sanity check: if either the color stream or the depth stream were not opened, return black image
  if (!streams_.count(openni::SENSOR_COLOR) || !streams_.count(openni::SENSOR_DEPTH)) return res_image;

  for (int y = 0; y < color_image.rows; ++y) {
    const uint16_t * const depth_row = depth_image.ptr<uint16_t>(y);
    for (int x = 0; x < color_image.cols; ++x) {
      // use white as 'no depth color'
      cv::Vec3b color(255,255,255);
      const uint16_t depth_value = depth_row[x];

      if (depth_value > 0) {
        int color_x, color_y;

        openni::CoordinateConverter::convertDepthToColor(
              *streams_.at(openni::SENSOR_DEPTH),
              *streams_.at(openni::SENSOR_COLOR),
              x, y,
              depth_value,
              &color_x, &color_y);

        if (color_x >= 0 && color_x < color_image.cols && color_y >= 0 && color_y < color_image.rows) {
          color = color_image.at<cv::Vec3b>(color_y, color_x);
        }
      }
      res_image.at<cv::Vec3b>(y,x) = color;
    }
  }

  return res_image;
}

cv::Mat GenericOpenNIDevice::convertMillimeterMatToPoint3fMat(const cv::Mat &depth_image) {
  const float millimeter_to_meter_factor = 1. / 1000.;

  cv::Mat_<cv::Point3f> res_image(depth_image.size());

  const auto fov = getDepthFov();

  const float xz_factor = tan(fov.first/2.) * 2.;
  const float yz_factor = tan(fov.second/2.) * 2.;

  for (int y = 0; y < res_image.cols; ++y) {
      const uint16_t *depth_row = depth_image.ptr<uint16_t>(y);

      cv::Point3f *target_row = res_image.ptr<cv::Point3f>(y);

      const float normalized_y = 0.5f - static_cast<float>(y) / static_cast<float>(res_image.cols);

      for (int x = 0; x < res_image.rows; ++x) {
        const float normalized_x = static_cast<float>(x) / static_cast<float>(res_image.rows) - 0.5f;

        const float depth_scaled = depth_row[x] * millimeter_to_meter_factor;
        target_row[x].x = normalized_x * depth_scaled * xz_factor;
        target_row[x].y = normalized_y * depth_scaled * yz_factor;
        target_row[x].z = -depth_scaled;
      }
  }

  return res_image;
}
