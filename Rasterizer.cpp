#include "Rasterizer.h"

Rasterizer::Rasterizer(int width, int height) : width(width), height(height) {
  image = cv::Mat::zeros(height, width, CV_8UC3);
}

void Rasterizer::clear() { image.setTo(cv::Scalar(0, 0, 0)); }

void Rasterizer::display(const std::string &windowName) {
  cv::imshow(windowName, image);
  cv::waitKey(0);
}

void Rasterizer::DrawLine(cv::Point start, cv::Point end, const cv::Vec3b &color) {
  cv::line(image, start, end, color, 1);
}

void Rasterizer::setPixel(int x, int y, const cv::Vec3b &color) {
  if (x >= 0 && x < width && y >= 0 && y < height) {
    image.at<cv::Vec3b>(y, x) = color;
  }
}