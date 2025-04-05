#include "Rasterizer.h"
#include "Triangle.h"
#include "global.h"
#include <fstream>
#include <string>

Rasterizer::Rasterizer(int _width, int _height)
    : width(_width), height(_height), zbuffer(_width * _height, std::numeric_limits<float>::min()) {
  image = cv::Mat::zeros(height, width, CV_8UC3);
}

void Rasterizer::clear() {
  image.setTo(cv::Scalar(0, 0, 0));
  zbuffer.assign(width * height, std::numeric_limits<float>::min());
}

void Rasterizer::display(const std::string &windowName) { cv::imshow(windowName, image); }

void Rasterizer::SetModelMatrix(const cv::Matx<double, 4, 4> &modelMatrix) { this->modelMatrix = cv::Mat(modelMatrix); }

void Rasterizer::SetViewMatrix(const cv::Matx<double, 4, 4> &viewMatrix) { this->viewMatrix = cv::Mat(viewMatrix); }

void Rasterizer::SetProjectionMatrix(const cv::Matx<double, 4, 4> &projectionMatrix) {
  this->projectionMatrix = cv::Mat(projectionMatrix);
}

void Rasterizer::AddTriangleFromObjWithTexture(const std::string &filename, const std::string &texturePath) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + filename);
  }
  cv::Mat texture = cv::imread(texturePath, cv::IMREAD_COLOR);
  if (texture.empty()) {
    throw std::runtime_error("Failed to load texture: " + texturePath);
  }

  std::vector<cv::Vec3d> vertexes;
  std::vector<std::array<cv::Vec3d, 3>> colors;
  std::vector<cv::Vec3d> normals;
  std::vector<cv::Vec2d> textureCoords;

  std::string line;
  while (std::getline(file, line)) {
    // Process each line here
    // For example, you can parse the line to extract vertex data
    std::istringstream iss(line);
    std::string prefix;
    iss >> prefix;

    if (prefix == "v") {
      cv::Vec3d vertex;
      iss >> vertex[0] >> vertex[1] >> vertex[2];
      // vertex *= 10;
      vertexes.push_back(vertex);
    } else if (prefix == "vn") {
      cv::Vec3d normal;
      iss >> normal[0] >> normal[1] >> normal[2];
      normals.push_back(normal);
    } else if (prefix == "vt") {
      cv::Vec2d texCoord;
      iss >> texCoord[0] >> texCoord[1];
      textureCoords.push_back(texCoord);
    } else if (prefix == "f") {

      Triangle triangle;
      std::array<cv::Vec3d, 3> faceVertexes;
      std::array<cv::Vec3d, 3> faceNormals;
      std::array<cv::Vec2d, 3> faceTextureCoords;
      std::array<cv::Vec3d, 3> faceColors;

      for (int i = 0; i < 3; ++i) {
        int index0, index1, index2;
        char slash;
        iss >> index0 >> slash >> index1 >> slash >> index2;
        faceVertexes[i] = vertexes[index0 - 1];
        faceNormals[i] = normals[index1 - 1];
        faceTextureCoords[i] = textureCoords[index2 - 1];
        // get color
        double u = std::clamp(faceTextureCoords[i][0], 0.0, 1.0);
        double v = std::clamp(faceTextureCoords[i][1], 0.0, 1.0);
        v = 1.0 - v;
        int x = static_cast<int>(u * (texture.cols - 1));
        int y = static_cast<int>(v * (texture.rows - 1));

        for (int i = 0; i < 3; ++i) {
          faceColors[i][0] = texture.at<cv::Vec3b>(y, x)[0] / 255.;
          faceColors[i][1] = texture.at<cv::Vec3b>(y, x)[1] / 255.;
          faceColors[i][2] = texture.at<cv::Vec3b>(y, x)[2] / 255.;
        }
      }
      triangle.SetVertexes(faceVertexes);
      triangle.SetNormals(faceNormals);
      triangle.SetTextureCoords(faceTextureCoords);
      triangle.SetColors(faceColors);
      triangles.push_back(triangle);
    }
  }
}

void Rasterizer::rasterize() {
  double f1 = (global::far - global::near) / 2.0;
  double f2 = (global::far + global::near) / 2.0;
  auto mvp = projectionMatrix * viewMatrix * modelMatrix;

  for (const auto &tri : triangles) {
    Triangle newtri = tri;

    std::array<cv::Vec4d, 3> mm;
    mm[0] = viewMatrix * modelMatrix * tri.a4();
    mm[1] = viewMatrix * modelMatrix * tri.b4();
    mm[2] = viewMatrix * modelMatrix * tri.c4();
    std::array<cv::Vec3d, 3> viewspace_pos;
    std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](cv::Vec4d &v) { return cv::Vec3d(v[0], v[1], v[2]); });
  }
}

void Rasterizer::setPixel(int x, int y, const cv::Vec3d &color) {
  if (x >= 0 && x < width && y >= 0 && y < height)
    image.at<cv::Vec3b>(y, x) =
        cv::Vec3b(static_cast<uchar>(color[0] * 255), static_cast<uchar>(color[1] * 255), static_cast<uchar>(color[2] * 255));
}
