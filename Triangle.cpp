#include "Triangle.h"

Triangle::Triangle(std::array<cv::Point3d, 3> &&_vertexes, std::array<cv::Vec3b, 3> &&_colors)
    : vertexes(std::move(_vertexes)), colors(std::move(_colors)) {}

// default color is white, so we can see it on the screen for test
Triangle::Triangle() { colors = {cv::Vec3b(255, 255, 255), cv::Vec3b(255, 255, 255), cv::Vec3b(255, 255, 255)}; }
