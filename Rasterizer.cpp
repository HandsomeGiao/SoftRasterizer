#include "Rasterizer.h"
#include "Triangle.h"
#include "global.h"
#include <fstream>
#include <string>

Rasterizer::Rasterizer(int _width, int _height)
    : width(_width), height(_height), zbuffer(_width * _height, std::numeric_limits<float>::max()) {
  image = cv::Mat::zeros(height, width, CV_8UC3);
}

void Rasterizer::clear() {
  image.setTo(cv::Scalar(0, 0, 0));
  zbuffer.assign(width * height, std::numeric_limits<float>::max());
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
  texture = cv::imread(texturePath);
  if (texture.empty()) {
    throw std::runtime_error("Failed to load texture: " + texturePath);
  }
  cv::cvtColor(texture, texture, cv::COLOR_RGB2BGR);

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

      for (int i = 0; i < 3; ++i) {
        int index0, index1, index2;
        char slash;
        iss >> index0 >> slash >> index1 >> slash >> index2;
        faceVertexes[i] = vertexes[index0 - 1];
        faceNormals[i] = normals[index1 - 1];
        faceTextureCoords[i] = textureCoords[index2 - 1];
      }
      triangle.SetVertexes(faceVertexes);
      triangle.SetNormals(faceNormals);
      triangle.SetTextureCoords(faceTextureCoords);
      triangles.push_back(triangle);
    }
  }
}

void Rasterizer::rasterize() {
  double f1 = (global::far - global::near) / 2.0;
  double f2 = (global::far + global::near) / 2.0;
  auto mvp = projectionMatrix * viewMatrix * modelMatrix;

  for (const auto &tri : triangles) {
    Triangle tri_clipspace;
    tri_clipspace.SetTextureCoords(tri.getTextureCoords());

    std::array<cv::Vec4d, 3> mm;
    mm[0] = viewMatrix * modelMatrix * tri.a4();
    mm[1] = viewMatrix * modelMatrix * tri.b4();
    mm[2] = viewMatrix * modelMatrix * tri.c4();
    std::array<cv::Vec3d, 3> viewspace_pos;
    std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](const cv::Vec4d &v) { return cv::Vec3d(v[0], v[1], v[2]); });

    std::array<cv::Vec4d, 3> pixelPos{mvp * tri.a4(), mvp * tri.b4(), mvp * tri.c4()};
    for (int i = 0; i < 3; ++i) {
      pixelPos[i] /= pixelPos[i][3];
      pixelPos[i][0] = 0.5 * global::width * (pixelPos[i][0] + 1.0);
      pixelPos[i][1] = 0.5 * global::height * (pixelPos[i][1] + 1.0);
      pixelPos[i][2] = f1 * pixelPos[i][2] + f2;
    }
    tri_clipspace.SetVertexes({cv::Vec3d(pixelPos[0][0], pixelPos[0][1], pixelPos[0][2]),
                               cv::Vec3d(pixelPos[1][0], pixelPos[1][1], pixelPos[1][2]),
                               cv::Vec3d(pixelPos[2][0], pixelPos[2][1], pixelPos[2][2])});

    cv::Matx<double, 4, 4> inv_t_mv = (viewMatrix * modelMatrix).inv().t();
    std::array<cv::Vec4d, 3> clipspace_normal;
    clipspace_normal[0] = inv_t_mv * tri.an4();
    // clipspace_normal[0] /= clipspace_normal[0][3];
    clipspace_normal[1] = inv_t_mv * tri.bn4();
    // clipspace_normal[1] /= clipspace_normal[1][3];
    clipspace_normal[2] = inv_t_mv * tri.cn4();
    // clipspace_normal[2] /= clipspace_normal[2][3];
    //  std::cout << clipspace_normal[0] << std::endl;

    tri_clipspace.SetNormals({cv::Vec3d(clipspace_normal[0][0], clipspace_normal[0][1], clipspace_normal[0][2]),
                              cv::Vec3d(clipspace_normal[1][0], clipspace_normal[1][1], clipspace_normal[1][2]),
                              cv::Vec3d(clipspace_normal[2][0], clipspace_normal[2][1], clipspace_normal[2][2])});
    // ??? set color
    rasterizeTriangle(tri_clipspace, viewspace_pos);
  }
}

void Rasterizer::setPixel(int x, int y, const cv::Vec3d &_color) {
  if (x >= 0 && x < width && y >= 0 && y < height) {
    cv::Vec3d color{std::clamp(_color[0], 0.0, 1.0), std::clamp(_color[1], 0.0, 1.0), std::clamp(_color[2], 0.0, 1.0)};
    color[0] = std::clamp(color[0], 0.0, 1.0);
    color[1] = std::clamp(color[1], 0.0, 1.0);
    color[2] = std::clamp(color[2], 0.0, 1.0);
    image.at<cv::Vec3b>(y, x) =
        cv::Vec3b(static_cast<uchar>(color[0] * 255), static_cast<uchar>(color[1] * 255), static_cast<uchar>(color[2] * 255));
  }
}

void Rasterizer::rasterizeTriangle(const Triangle &triangle_clipspace, const std::array<cv::Vec3d, 3> &viewspace_pos) {
  cv::Vec2i topleft, bottomright;
  topleft[0] = std::max({triangle_clipspace.a()[1], triangle_clipspace.b()[1], triangle_clipspace.c()[1]});
  topleft[1] = std::min({triangle_clipspace.a()[0], triangle_clipspace.b()[0], triangle_clipspace.c()[0]});
  bottomright[0] = std::min({triangle_clipspace.a()[1], triangle_clipspace.b()[1], triangle_clipspace.c()[1]});
  bottomright[1] = std::max({triangle_clipspace.a()[0], triangle_clipspace.b()[0], triangle_clipspace.c()[0]});

  for (int y = bottomright[0]; y <= topleft[0]; y++) {
    for (int x = topleft[1]; x <= bottomright[1]; x++) {
      if (insideTriangle(triangle_clipspace, x, y)) {
        auto [alpha, beta, gamma] = computeBarycentric2D(x, y, triangle_clipspace.getVertexes());
        float Z = 1. / (alpha + beta + gamma);
        float zp = alpha * triangle_clipspace.a()[2] + beta * triangle_clipspace.b()[2] + gamma * triangle_clipspace.c()[2];
        zp *= Z;
        if (zp < zbuffer[y * global::width + x]) {
          zbuffer[y * global::width + x] = zp;
          auto interp_normal = alpha * triangle_clipspace.an() + beta * triangle_clipspace.bn() + gamma * triangle_clipspace.cn();
          auto interp_textureCoord = alpha * triangle_clipspace.getTextureCoords()[0] +
                                     beta * triangle_clipspace.getTextureCoords()[1] +
                                     gamma * triangle_clipspace.getTextureCoords()[2];
          auto interp_shadingCoord = alpha * viewspace_pos[0] + beta * viewspace_pos[1] + gamma * viewspace_pos[2];
          auto color = BillnPhongShading(interp_shadingCoord, cv::normalize(interp_normal), interp_textureCoord);
          setPixel(x, y, color);
        }
      }
    }
  }
}

bool Rasterizer::insideTriangle(const Triangle &tri_clipspace, int x, int y) const {
  cv::Vec3d v[3];
  v[0] = cv::Vec3d(tri_clipspace.a()[0], tri_clipspace.a()[1], 1.);
  v[1] = cv::Vec3d(tri_clipspace.b()[0], tri_clipspace.b()[1], 1.);
  v[2] = cv::Vec3d(tri_clipspace.c()[0], tri_clipspace.c()[1], 1.);

  cv::Vec3d f0, f1, f2;
  f0 = v[1].cross(v[0]);
  f1 = v[2].cross(v[1]);
  f2 = v[0].cross(v[2]);
  cv::Vec3d p(x, y, 1.);
  if ((p.dot(f0) * f0.dot(v[2]) > 0) && (p.dot(f1) * f1.dot(v[0]) > 0) && (p.dot(f2) * f2.dot(v[1]) > 0))
    return true;
  return false;
}

std::tuple<float, float, float> Rasterizer::computeBarycentric2D(float x, float y, const std::array<cv::Vec3d, 3> v) {
  float c1 = (x * (v[1][1] - v[2][1]) + (v[2][0] - v[1][0]) * y + v[1][0] * v[2][1] - v[2][0] * v[1][1]) /
             (v[0][0] * (v[1][1] - v[2][1]) + (v[2][0] - v[1][0]) * v[0][1] + v[1][0] * v[2][1] - v[2][0] * v[1][1]);
  float c2 = (x * (v[2][1] - v[0][1]) + (v[0][0] - v[2][0]) * y + v[2][0] * v[0][1] - v[0][0] * v[2][1]) /
             (v[1][0] * (v[2][1] - v[0][1]) + (v[0][0] - v[2][0]) * v[1][1] + v[2][0] * v[0][1] - v[0][0] * v[2][1]);
  float c3 = (x * (v[0][1] - v[1][1]) + (v[1][0] - v[0][0]) * y + v[0][0] * v[1][1] - v[1][0] * v[0][1]) /
             (v[2][0] * (v[0][1] - v[1][1]) + (v[1][0] - v[0][0]) * v[2][1] + v[0][0] * v[1][1] - v[1][0] * v[0][1]);
  return {c1, c2, c3};
}

// return [0,255.]
cv::Vec3d Rasterizer::getTexColor(double u, double v) const {
  u = std::clamp(u, 0., 1.);
  v = std::clamp(v, 0., 1.);
  return cv::Vec3d(
      texture.at<cv::Vec3b>(static_cast<int>((1 - v) * (texture.rows - 1)), static_cast<int>(u * (texture.cols - 1))));
}

cv::Vec3d Rasterizer::BillnPhongShading(const cv::Vec3d &point, const cv::Vec3d &normal, const cv::Vec2d texCoord) {
  cv::Vec3d ka = cv::Vec3d(0.005, 0.005, 0.005);               // Ambient light color
  cv::Vec3d kd = getTexColor(texCoord[0], texCoord[1]) / 255.; // Diffuse light color
  // std::cout << "kd: " << kd << std::endl;
  // std::cout << "coord: " << texCoord[0] << " " << texCoord[1] << std::endl;
  cv::Vec3d ks = cv::Vec3d(0.7937, 0.7937, 0.7937); // Specular light color

  cv::Vec3d result_color = {0., 0., 0.};
  for (const auto &light : lights) {
    // diffuse
    cv::Vec3d L = cv::normalize(light.position - point);
    double l_dot_n = std::max(0., normal.dot(L));
    cv::Vec3d diffuse = kd.mul(light.intensity / cv::norm(light.position - point, cv::NORM_L2SQR)) * l_dot_n;
    // specular
    cv::Vec3d h = cv::normalize(L + cv::normalize(global::eye - point));
    double h_dot_n = std::max(0., normal.dot(h));
    cv::Vec3d specular = ks.mul(light.intensity / cv::norm(light.position - point, cv::NORM_L2SQR)) * std::pow(h_dot_n, 300);
    // ambient
    cv::Vec3d ambient = ka.mul(global::ambientLight);
    // accumulate
    result_color += diffuse + specular + ambient;
  }

  return result_color;
}
