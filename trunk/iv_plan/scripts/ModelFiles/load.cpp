#include <iostream>
#include <cv.h>
#include "opencv2/opencv.hpp"
#include <time.h>

using namespace std;
using namespace cv;
 
int
main (int argc, char **argv)
{
  FileStorage fs2("parts.yml", FileStorage::READ);

  // first method: use (type) operator on FileNode.

  std::string Name;
  // second method: use FileNode::operator >>
  fs2["ModelName"] >> Name;

  Mat vertices, edges, circles;
  fs2["vertices"] >> vertices;
  fs2["edges"] >> edges;
  fs2["circles"] >> circles;

  cout << "ModelName: " << Name << endl
       << "vertices: " << vertices << endl
       << "edges: " << edges << endl
       << "circles: " << circles << endl;

  fs2.release();

  return 0;
}
