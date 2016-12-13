#pragma once

// these are the possible image types a generic openni device can provide
enum class ImageType {
  DEPTH_MM,
  DEPTH_GRAYSCALE,
  POINT_3F,
  RGB,
  RGB_ALIGNED,
  IR,
  IR_GRAYSCALE,
};

