#include "Triangle.h"

Triangle::Triangle(std::array<cv::Vec3d, 3> &&_vertexes, std::array<cv::Vec3d, 3> &&_colors)
    : vertexes(std::move(_vertexes)), colors(std::move(_colors)) {}

Triangle::Triangle() {}
