#include "global.h"

namespace global {
double near = 0.1;
double far = 100.0;
int width = 800;
int height = 600;
cv::Vec3d eye(0.0, 0.0, 10.0);
cv::Vec3d center(0.0, 0.0, 0.0);
cv::Vec3d eyeup(0.0, -1.0, 0.0);
double fov = 45.0;
double aspect = static_cast<double>(width) / height;
cv::Vec3d ambientLight(10., 10., 10.);
} // namespace global
