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
  inline void writeImage(const std::string &filename) const { cv::imwrite(filename, image); }

private:
  cv::Mat image;
  int width, height;

  std::vector<Triangle> triangles;
  std::vector<Light> lights;
  std::vector<double> zbuffer;

  cv::Matx<double, 4, 4> modelMatrix = cv::Matx<double, 4, 4>::eye();      // Initialize to identity matrix
  cv::Matx<double, 4, 4> viewMatrix = cv::Matx<double, 4, 4>::eye();       // Initialize to identity matrix
  cv::Matx<double, 4, 4> projectionMatrix = cv::Matx<double, 4, 4>::eye(); // Initialize to identity matrix

  void setPixel(int x, int y, const cv::Vec3d &color);
  void rasterizeTriangle(const Triangle &triangle, const std::array<cv::Vec3d, 3> &viewspace_pos);
  bool insideTriangle(const Triangle &tri_clipspace, int x, int y) const;
  std::tuple<float, float, float> computeBarycentric2D(float x, float y, const std::array<cv::Vec3d, 3> v);
  cv::Vec3d BillnPhongShading(const cv::Vec3d color, const cv::Vec3d &viewPos, const cv::Vec3d &normal, const cv::Vec2d texCoord);
};
