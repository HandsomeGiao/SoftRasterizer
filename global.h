#pragma once

#include <opencv2/core.hpp>

namespace global {
extern double near;
extern double far;
extern int width;
extern int height;
extern cv::Vec3d eye;
extern cv::Vec3d center;
extern cv::Vec3d eyeup;
extern double fov;
extern double aspect;
} // namespace global