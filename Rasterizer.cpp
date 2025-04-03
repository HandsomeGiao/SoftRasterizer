#include "Rasterizer.h"
#include "Triangle.h"
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

void Rasterizer::AddTriangle(const Triangle &t) { triangles.push_back(t); }

void Rasterizer::AddTriangleFromObjWithTexture(const std::string &filename, const std::string &texturePath) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + filename);
  }
  cv::Mat texture = cv::imread(texturePath, cv::IMREAD_COLOR);
  if (texture.empty()) {
    throw std::runtime_error("Failed to load texture: " + texturePath);
  }

  std::vector<cv::Point3d> vertexes;
  std::vector<std::array<cv::Vec3b, 3>> colors;
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
      cv::Point3d vertex;
      iss >> vertex.x >> vertex.y >> vertex.z;
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
      std::array<cv::Point3d, 3> faceVertexes;
      std::array<cv::Vec3d, 3> faceNormals;
      std::array<cv::Vec2d, 3> faceTextureCoords;
      std::array<cv::Vec3b, 3> faceColors;

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

        faceColors[i] = texture.at<cv::Vec3b>(y, x);
      }
      triangle.SetVertexes(faceVertexes);
      triangle.SetNormals(faceNormals);
      triangle.SetTextureCoords(faceTextureCoords);
      triangle.SetColors(faceColors);
      triangles.push_back(triangle);
    }
  }
}

void Rasterizer::RasterizeAllTriangleWithInterplate() {
  for (const auto &tri : triangles) {
    auto ta = TransformVertex(tri.a());
    auto tb = TransformVertex(tri.b());
    auto tc = TransformVertex(tri.c());

    auto toPixel = [this](const cv::Point3d &p) {
      return cv::Point(static_cast<int>((p.x + 1.) * 0.5 * width), static_cast<int>((p.y + 1.) * 0.5 * height));
    };

    cv::Point pixel1 = toPixel(ta);
    cv::Point pixel2 = toPixel(tb);
    cv::Point pixel3 = toPixel(tc);

    int minX = std::min({pixel1.x, pixel2.x, pixel3.x});
    minX = std::max(0, minX);
    int maxX = std::max({pixel1.x, pixel2.x, pixel3.x});
    maxX = std::min(width - 1, maxX);
    int minY = std::min({pixel1.y, pixel2.y, pixel3.y});
    minY = std::max(0, minY);
    int maxY = std::max({pixel1.y, pixel2.y, pixel3.y});
    maxY = std::min(height - 1, maxY);

    for (int x = minX; x <= maxX; ++x) {
      for (int y = minY; y <= maxY; ++y) {
        float alpha, beta, gamma;
        if (ComputeBarycentric(pixel1, pixel2, pixel3, cv::Point(x, y), alpha, beta, gamma)) {
          float zinterp = alpha * ta.z + beta * tb.z + gamma * tc.z;
          if (zbuffer[y * width + x] < zinterp) {
            zbuffer[y * width + x] = zinterp;
            cv::Vec3b color = alpha * tri.ac() + beta * tri.bc() + gamma * tri.cc();
            setPixel(x, y, color);
          }
        }
      }
    }
    // DrawGradientTriangle(pixel1, pixel2, pixel3, tri.ac(), tri.bc(), tri.cc());
  }
}

void Rasterizer::RasterizaAllTriangle() {

  for (const auto &tri : triangles) {
    auto ta = TransformVertex(tri.a());
    auto tb = TransformVertex(tri.b());
    auto tc = TransformVertex(tri.c());

    auto toPixel = [this](const cv::Point3d &p) {
      return cv::Point(static_cast<int>((p.x + 1.) * 0.5 * width), static_cast<int>((p.y + 1.) * 0.5 * height));
    };

    cv::Point pixel1 = toPixel(ta);
    cv::Point pixel2 = toPixel(tb);
    cv::Point pixel3 = toPixel(tc);

    DrawTriangle(pixel1, pixel2, pixel3, tri.ac());
  }
}

void Rasterizer::DrawLine(cv::Point start, cv::Point end, const cv::Vec3b &color) { cv::line(image, start, end, color, 1); }

void Rasterizer::DrawTriangle(cv::Point p1, cv::Point p2, cv::Point p3, const cv::Vec3b &color) {
  cv::line(image, p1, p2, color, 1);
  cv::line(image, p2, p3, color, 1);
  cv::line(image, p3, p1, color, 1);
}

void Rasterizer::DrawFullTriangle(cv::Point p1, cv::Point p2, cv::Point p3, const cv::Vec3b &color) {
  cv::fillConvexPoly(image, std::vector<cv::Point>{p1, p2, p3}, color);
}

void Rasterizer::DrawGradientTriangle(cv::Point p1, cv::Point p2, cv::Point p3, const cv::Vec3b &color1, const cv::Vec3b &color2,
                                      const cv::Vec3b &color3) {
  int minX = std::min({p1.x, p2.x, p3.x});
  minX = std::max(0, minX);
  int maxX = std::max({p1.x, p2.x, p3.x});
  maxX = std::min(width - 1, maxX);
  int minY = std::min({p1.y, p2.y, p3.y});
  minY = std::max(0, minY);
  int maxY = std::max({p1.y, p2.y, p3.y});
  maxY = std::min(height - 1, maxY);

  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      float alpha, beta, gamma;
      if (ComputeBarycentric(p1, p2, p3, cv::Point(x, y), alpha, beta, gamma)) {
        cv::Vec3b color = alpha * color1 + beta * color2 + gamma * color3;
        setPixel(x, y, color);
      }
    }
  }
}

bool Rasterizer::ComputeBarycentric(cv::Point p1, cv::Point p2, cv::Point p3, cv::Point p, float &alpha, float &beta,
                                    float &gamma) {
  float area = (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);
  if (std::abs(area) < 1e-5)
    return false;

  alpha = ((p2.x - p.x) * (p3.y - p.y) - (p3.x - p.x) * (p2.y - p.y)) / area;
  beta = ((p3.x - p.x) * (p1.y - p.y) - (p1.x - p.x) * (p3.y - p.y)) / area;
  gamma = 1.0f - alpha - beta;

  return alpha >= 0 && beta >= 0 && gamma >= 0;
}

cv::Point3d Rasterizer::TransformVertex(const cv::Point3d &vertex) {
  cv::Mat vertexH = (cv::Mat_<double>(4, 1) << vertex.x, vertex.y, vertex.z, 1.0);
  cv::Mat transformedH = projectionMatrix * viewMatrix * modelMatrix * vertexH;
  double w = transformedH.at<double>(3, 0);

  return cv::Point3d(transformedH.at<double>(0, 0) / w, transformedH.at<double>(1, 0) / w, transformedH.at<double>(2, 0) / w);
}

void Rasterizer::setPixel(int x, int y, const cv::Vec3b &color) {
  if (x >= 0 && x < width && y >= 0 && y < height) {
    image.at<cv::Vec3b>(y, x) = color;
  }
}