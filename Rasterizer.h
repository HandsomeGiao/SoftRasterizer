#pragma once

#include "Light.h"
#include "Triangle.h"
#include <opencv2/opencv.hpp>

class Rasterizer {
public:
  Rasterizer(int width, int height);

  void clear();
  void display(const std::string &windowName);

  void SetModelMatrix(const cv::Matx<double, 4, 4> &modelMatrix);
  void SetViewMatrix(const cv::Matx<double, 4, 4> &viewMatrix);
  void SetProjectionMatrix(const cv::Matx<double, 4, 4> &projectionMatrix);

  void AddTriangleFromObjWithTexture(const std::string &filename, const std::string &texturePath);
  inline void AddLight(const Light &light) { lights.push_back(light); }

  void rasterize();

private:
  cv::Mat image;
  int width, height;

  std::vector<Triangle> triangles;
  std::vector<Light> lights;
  std::vector<double> zbuffer;

  cv::Mat modelMatrix = cv::Mat::eye(4, 4, CV_64F);      // Initialize to identity matrix
  cv::Mat viewMatrix = cv::Mat::eye(4, 4, CV_64F);       // Initialize to identity matrix
  cv::Mat projectionMatrix = cv::Mat::eye(4, 4, CV_64F); // Initialize to identity matrix

  void setPixel(int x, int y, const cv::Vec3d &color);
};
