#pragma once

#include <opencv2/opencv.hpp>

// 代表一个空间中的三角形
class Triangle {
public:
  Triangle(std::array<cv::Point3d, 3> &&_vertexes, std::array<cv::Vec3b, 3> &&_colors);
  Triangle();

  inline void SetVertexes(const std::array<cv::Point3d, 3> &_vertexes) { this->vertexes = _vertexes; }
  inline void SetColors(const std::array<cv::Vec3b, 3> &_colors) { this->colors = _colors; }
  inline void SetNormals(const std::array<cv::Vec3d, 3> &_normals) { this->normals = _normals; }
  inline void SetTextureCoords(const std::array<cv::Vec2d, 3> &_textureCoords) { this->textureCoords = _textureCoords; }

  inline cv::Point3d a() const { return vertexes[0]; }
  inline cv::Point3d b() const { return vertexes[1]; }
  inline cv::Point3d c() const { return vertexes[2]; }
  inline cv::Vec3b ac() const { return colors[0]; }
  inline cv::Vec3b bc() const { return colors[1]; }
  inline cv::Vec3b cc() const { return colors[2]; }

private:
  std::array<cv::Point3d, 3> vertexes;
  std::array<cv::Vec3b, 3> colors;
  std::array<cv::Vec3d, 3> normals;
  std::array<cv::Vec2d, 3> textureCoords;
};
