#pragma once

#include <opencv2/opencv.hpp>

class Triangle;

class Rasterizer {
public:
  Rasterizer(int width, int height);

  void clear();
  void display(const std::string &windowName);

  void SetModelMatrix(const cv::Matx<double, 4, 4> &modelMatrix);
  void SetViewMatrix(const cv::Matx<double, 4, 4> &viewMatrix);
  void SetProjectionMatrix(const cv::Matx<double, 4, 4> &projectionMatrix);

  void AddTriangle(const Triangle &t);
  void Rasterize();

private:
  cv::Mat image;
  int width, height;

  std::vector<Triangle> triangles;
  std::vector<double> zbuffer;

  cv::Mat modelMatrix = cv::Mat::eye(4, 4, CV_64F);      // Initialize to identity matrix
  cv::Mat viewMatrix = cv::Mat::eye(4, 4, CV_64F);       // Initialize to identity matrix
  cv::Mat projectionMatrix = cv::Mat::eye(4, 4, CV_64F); // Initialize to identity matrix

  void setPixel(int x, int y, const cv::Vec3b &color);
  void DrawLine(cv::Point start, cv::Point end, const cv::Vec3b &color);
  void DrawTriangle(cv::Point p1, cv::Point p2, cv::Point p3, const cv::Vec3b &color);
  void DrawFullTriangle(cv::Point p1, cv::Point p2, cv::Point p3, const cv::Vec3b &color);
  void DrawGradientTriangle(cv::Point p1, cv::Point p2, cv::Point p3, const cv::Vec3b &color1, const cv::Vec3b &color2, const cv::Vec3b &color3);
  bool ComputeBarycentric(cv::Point p1, cv::Point p2, cv::Point p3, cv::Point p, float &alpha, float &beta, float &gamma);

  cv::Point3d TransformVertex(const cv::Point3d &vertex);
};
