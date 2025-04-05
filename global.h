#pragma once

#include <opencv2/core.hpp>

int width = 800, height = 600;
cv::Vec3d eye(0., 0., 2.);
cv::Vec3d center(0., 0., 0.);
cv::Vec3d eyeup{0., -1., 0.};

double fov = 45.0;
double aspect = static_cast<double>(width) / height;
double near = 0.1, far = 100.0;
