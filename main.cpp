#include "Rasterizer.h"
#include "Triangle.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

Mat GetViewMatrix(Vec3d eye, Vec3d center, Vec3d up) {
  Vec3d z = eye - center;
  z = normalize(z);
  Vec3d x = up.cross(z);
  x = normalize(x);
  Vec3d y = z.cross(x);
  y = normalize(y);

  Mat view = Mat::eye(4, 4, CV_64F);
  view.at<double>(0, 0) = x[0];
  view.at<double>(0, 1) = x[1];
  view.at<double>(0, 2) = x[2];
  view.at<double>(1, 0) = y[0];

  view.at<double>(1, 1) = y[1];
  view.at<double>(1, 2) = y[2];
  view.at<double>(2, 0) = z[0];
  view.at<double>(2, 1) = z[1];
  view.at<double>(2, 2) = z[2];

  view.at<double>(0, 3) = -x.dot(eye);
  view.at<double>(1, 3) = -y.dot(eye);
  view.at<double>(2, 3) = -z.dot(eye);

  return view;
}

Mat GetPerspectiveProjectionMatrix(double fov, double aspect, double near, double far) {
  double rad = fov * CV_PI / 180.0;
  double tanHalfFov = tan(rad / 2.0);

  cv::Mat projection = cv::Mat::zeros(4, 4, CV_64F);
  projection.at<double>(0, 0) = 1.0 / (aspect * tanHalfFov);
  projection.at<double>(1, 1) = 1.0 / tanHalfFov;
  projection.at<double>(2, 2) = -(far + near) / (far - near);
  projection.at<double>(2, 3) = -2.0 * far * near / (far - near);
  projection.at<double>(3, 2) = -1.0;

  return projection;
}

int main() {
  int width = 800, height = 600;
  Rasterizer rasterizer(width, height);

  Mat model = Mat::eye(4, 4, CV_64F);
  // Get View
  Vec3d eye(0., 0., 2.);
  Vec3d center(0., 0., 0.);
  Vec3d eyeup{0., -1., 0.};
  Mat view = GetViewMatrix(eye, center, eyeup);
  double fov = 45.0;
  double aspect = static_cast<double>(width) / height;
  double near = 0.1, far = 100.0;
  Mat projection = GetPerspectiveProjectionMatrix(fov, aspect, near, far);

  rasterizer.SetModelMatrix(model);
  rasterizer.SetViewMatrix(view);
  rasterizer.SetProjectionMatrix(projection);

  rasterizer.AddTriangleFromObjWithTexture("./models/cow/cow.obj", "./models/cow/cow_texture.png");

  rasterizer.rasterize();
  rasterizer.display("Line Drawing Example");

  return 0;
}
