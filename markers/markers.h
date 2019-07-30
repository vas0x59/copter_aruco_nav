#ifndef MARKERS_H_
#define MARKERS_H_

#include <iostream>
#include <vector>
#include <string>
#include "opencv2/opencv.hpp"
// #include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "math.h"
// class Marker{

// };
// struct Marker {
//     int id;

// };
using namespace std;
using namespace cv;

// struct Point3f{
//     float x;
//     float y;
//     float z;
// };
// using namespace markers;

namespace markers
{
void PRINT_DEBUG(string str);

// Point3f rotate_by_z(Point3f p1, Point3f p2){
//     Point3f p3 = p1;

//     p3.x = ((x1 - x0) * cos(a)) - ((y1 - y0) * sin(a)) + x0;
//     p3.y = ((x1 - x0) * sin(a)) + ((y1 - y0) * cos(a)) + y0;
// }
cv::Point3f rotate3d(const cv::Point3f inPoint, const cv::Point3f center, cv::Point3f rotation);

void alignObjPointsToCenter(Mat &obj_points, double &center_x, double &center_y, double &center_z);

struct Pose
{
    Point3f pose;
    Point3f rotation;
};

String string_pose(Pose pose);

Point3d rotationMatrixToEAngles(Mat R);
// cv::rot
Mat R_flip_gen(double x = 1.0, double y = -1.0, double z = -1.0);

} // namespace Markers
// void Solver{
// }
#endif
