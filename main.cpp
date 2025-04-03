#include <iostream>
#include <opencv2/opencv.hpp>

#include "Rasterizer.h"

using namespace std;
using namespace cv;

int main() {
  Rasterizer rasterizer(800, 600);
  rasterizer.DrawLine(Point(100, 100), Point(700, 500), Vec3b(255, 0, 0));
  rasterizer.display("Line Drawing Example");
  return 0;
}
