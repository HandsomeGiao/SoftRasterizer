#pragma once

#include <opencv2/opencv.hpp>

class Rasterizer {
public:
  Rasterizer(int width, int height);

  void clear();
  void display(const std::string &windowName);

  void DrawLine(cv::Point start, cv::Point end, const cv::Vec3b &color);

private:
  cv::Mat image;
  int width, height;

  void setPixel(int x, int y, const cv::Vec3b &color);
};
