#include "opencv2/opencv.hpp"
#include <time.h>

using namespace cv;

int main(int, char** argv)
{
  FileStorage fs("parts.yml", FileStorage::WRITE);

  fs << "ModelName" << "parts";
  Mat vertices = (Mat_<double>(4,3) << 25.000000, 20.000000, 15.000000, 25.000000, -20.000000, 15.000000, -25.000000, 20.000000, 15.000000, -25.000000, -20.000000, 15.000000);
  Mat edges = (Mat_<int>(4,2) <<  0, 1, 0, 2, 1, 3, 2, 3);
  Mat circles = (Mat_<double>(1,4) <<  0, 0, 15, 14);
  fs << "vertices" << vertices << "edges" << edges << "circles" << circles;
  fs.release();
  return 0;
}
