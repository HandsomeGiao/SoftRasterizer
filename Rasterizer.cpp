#include "Rasterizer.h"
#include "Triangle.h"

Rasterizer::Rasterizer(int _width, int _height) : width(_width), height(_height), zbuffer(_width * _height, std::numeric_limits<float>::max()) {
  image = cv::Mat::zeros(height, width, CV_8UC3);
}

void Rasterizer::clear() { image.setTo(cv::Scalar(0, 0, 0)); }

void Rasterizer::display(const std::string &windowName) {
  cv::imshow(windowName, image);
  cv::waitKey(0);
}

void Rasterizer::SetModelMatrix(const cv::Matx<double, 4, 4> &modelMatrix) { this->modelMatrix = cv::Mat(modelMatrix); }

void Rasterizer::SetViewMatrix(const cv::Matx<double, 4, 4> &viewMatrix) { this->viewMatrix = cv::Mat(viewMatrix); }

void Rasterizer::SetProjectionMatrix(const cv::Matx<double, 4, 4> &projectionMatrix) { this->projectionMatrix = cv::Mat(projectionMatrix); }

void Rasterizer::AddTriangle(const Triangle &t) { triangles.push_back(t); }

void Rasterizer::Rasterize() {
  for (const auto &tri : triangles) {
    auto ta = TransformVertex(tri.a());
    auto tb = TransformVertex(tri.b());
    auto tc = TransformVertex(tri.c());

    auto toPixel = [this](const cv::Point3d &p) { return cv::Point(static_cast<int>((p.x + 1.) * 0.5 * width), static_cast<int>((p.y + 1.) * 0.5 * height)); };

    cv::Point pixel1 = toPixel(ta);
    cv::Point pixel2 = toPixel(tb);
    cv::Point pixel3 = toPixel(tc);

    DrawGradientTriangle(pixel1, pixel2, pixel3, tri.ac(), tri.bc(), tri.cc());
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

void Rasterizer::DrawGradientTriangle(cv::Point p1, cv::Point p2, cv::Point p3, const cv::Vec3b &color1, const cv::Vec3b &color2, const cv::Vec3b &color3) {
  // 计算三角形的边界框
  int minX = std::min({p1.x, p2.x, p3.x});
  int maxX = std::max({p1.x, p2.x, p3.x});
  int minY = std::min({p1.y, p2.y, p3.y});
  int maxY = std::max({p1.y, p2.y, p3.y});

  // 遍历边界框内的每个像素
  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      // 计算当前像素点的重心坐标
      float alpha, beta, gamma;
      if (ComputeBarycentric(p1, p2, p3, cv::Point(x, y), alpha, beta, gamma)) {
        // 使用重心坐标插值颜色
        cv::Vec3b color = alpha * color1 + beta * color2 + gamma * color3;

        // 设置像素颜色
        setPixel(x, y, color);
      }
    }
  }
}

bool Rasterizer::ComputeBarycentric(cv::Point p1, cv::Point p2, cv::Point p3, cv::Point p, float &alpha, float &beta, float &gamma) {
  // 计算三角形面积
  float area = (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);
  if (std::abs(area) < 1e-5)
    return false; // 面积过小，视为无效

  // 计算重心坐标
  alpha = ((p2.x - p.x) * (p3.y - p.y) - (p3.x - p.x) * (p2.y - p.y)) / area;
  beta = ((p3.x - p.x) * (p1.y - p.y) - (p1.x - p.x) * (p3.y - p.y)) / area;
  gamma = 1.0f - alpha - beta;

  // 检查是否在三角形内部
  return alpha >= 0 && beta >= 0 && gamma >= 0;
}

cv::Point3d Rasterizer::TransformVertex(const cv::Point3d &vertex) {
  cv::Mat vertexH = (cv::Mat_<double>(4, 1) << vertex.x, vertex.y, vertex.z, 1.0);
  cv::Mat transformedH = projectionMatrix * viewMatrix * modelMatrix * vertexH;
  double w = transformedH.at<double>(3, 0);

  std::cout<<"transformedH: " << transformedH << std::endl;

  return cv::Point3d(transformedH.at<double>(0, 0) / w, transformedH.at<double>(1, 0) / w, transformedH.at<double>(2, 0) / w);
}

void Rasterizer::setPixel(int x, int y, const cv::Vec3b &color) {
  if (x >= 0 && x < width && y >= 0 && y < height) {
    image.at<cv::Vec3b>(y, x) = color;
  }
}