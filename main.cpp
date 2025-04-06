#include "Light.h"
#include "Rasterizer.h"
#include "Triangle.h"
#include "global.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

Mat GetModelMatrix() {
  double angle = -135 * 3.1415926 / 180.;
  Mat rotation = Mat::zeros(4, 4, CV_64F);
  rotation.at<double>(0, 0) = cos(angle);
  rotation.at<double>(0, 2) = sin(angle);
  rotation.at<double>(1, 1) = 1;
  rotation.at<double>(2, 0) = -sin(angle);
  rotation.at<double>(2, 2) = cos(angle);
  rotation.at<double>(3, 3) = 1;

  Mat scale = Mat::zeros(4, 4, CV_64F);
  scale.at<double>(0, 0) = 2.5;
  scale.at<double>(1, 1) = 2.5;
  scale.at<double>(2, 2) = 2.5;
  scale.at<double>(3, 3) = 1.0;

  Mat translate = Mat::eye(4, 4, CV_64F);

  return translate * rotation * scale;
}

Mat GetViewMatrix() {
  Vec3d z = global::eye - global::center;
  z = normalize(z);
  Vec3d x = global::eyeup.cross(z);
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

  view.at<double>(0, 3) = -x.dot(global::eye);
  view.at<double>(1, 3) = -y.dot(global::eye);
  view.at<double>(2, 3) = -z.dot(global::eye);

  return view;
}

Mat GetPerspectiveProjectionMatrix() {
  double rad = global::fov * CV_PI / 180.0;
  double tanHalfFov = tan(rad / 2.0);

  cv::Mat projection = cv::Mat::zeros(4, 4, CV_64F);
  projection.at<double>(0, 0) = 1.0 / (global::aspect * tanHalfFov);
  projection.at<double>(1, 1) = 1.0 / tanHalfFov;
  projection.at<double>(2, 2) = -(global::far + global::near) / (global::far - global::near);
  projection.at<double>(2, 3) = -2.0 * global::far * global::near / (global::far - global::near);
  projection.at<double>(3, 2) = -1.0;

  return projection;
}

int main() {
  Rasterizer rasterizer(global::width, global::height);

  Mat model = GetModelMatrix();
  Mat view = GetViewMatrix();
  Mat projection = GetPerspectiveProjectionMatrix();

  rasterizer.SetModelMatrix(model);
  rasterizer.SetViewMatrix(view);
  rasterizer.SetProjectionMatrix(projection);

  Light l1{{20, -20, 20}, {500, 500, 500}}, l2{{-20, -20, 0}, {500, 500, 500}};
  rasterizer.AddLight(l1);
  rasterizer.AddLight(l2);

  rasterizer.AddTriangleFromObjWithTexture("./models/cow/cow.obj", "./models/cow/cow_texture.png");

  rasterizer.rasterize();
  rasterizer.display("Line Drawing Example");
  rasterizer.writeImage("output.png");
  cv::waitKey(0);

  return 0;
}
