#pragma once

#include <opencv2/core.hpp>

struct Light {
  cv::Vec3d position;
  cv::Vec3d intensity;
};