#include "markers.h"
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
void PRINT_DEBUG(string str)
{
    cout << str << endl;
}

// Point3f rotate_by_z(Point3f p1, Point3f p2){
//     Point3f p3 = p1;

//     p3.x = ((x1 - x0) * cos(a)) - ((y1 - y0) * sin(a)) + x0;
//     p3.y = ((x1 - x0) * sin(a)) + ((y1 - y0) * cos(a)) + y0;
// }
cv::Point3f rotate3d(const cv::Point3f inPoint, const cv::Point3f center, cv::Point3f rotation)
{
    cv::Point3f point = inPoint - center;

    float temp = point.y;
    point.y = point.y * cos(rotation.x) - point.z * sin(rotation.x);
    point.z = temp * sin(rotation.x) + point.z * cos(rotation.x);

    temp = point.z;
    point.z = point.z * cos(rotation.y) - point.x * sin(rotation.y);
    point.x = temp * sin(rotation.y) + point.x * cos(rotation.y);

    temp = point.x;
    point.x = point.x * cos(rotation.z) - point.y * sin(rotation.z);
    point.y = temp * sin(rotation.z) + point.y * cos(rotation.z);

    return center + point;
    // cv::Point3f outPoint = inPoint2;
    // //CW rotation
    // // outPoint.x = std::cos(angRad)*inPoint.x - std::sin(angRad)*inPoint.y;
    // // outPoint.y = std::sin(angRad)*inPoint.x + std::cos(angRad)*inPoint.y;
    //
    // outPoint.x = ((inPoint.x - inPoint2.x) * cos(a)) - ((inPoint.y - inPoint2.y) * sin(a)) + inPoint2.x;
    // outPoint.y = ((inPoint.x - inPoint2.x) * sin(a)) + ((inPoint.y - inPoint2.y) * cos(a)) + inPoint2.y;
    // return outPoint;
}

void alignObjPointsToCenter(Mat &obj_points, double &center_x, double &center_y, double &center_z)
{
    // Align object points to the center of mass
    double sum_x = 0;
    double sum_y = 0;
    double sum_z = 0;

    for (int i = 0; i < obj_points.rows; i++)
    {
        sum_x += obj_points.at<float>(i, 0);
        sum_y += obj_points.at<float>(i, 1);
        sum_z += obj_points.at<float>(i, 2);
    }

    center_x = sum_x / obj_points.rows;
    center_y = sum_y / obj_points.rows;
    center_z = sum_z / obj_points.rows;

    for (int i = 0; i < obj_points.rows; i++)
    {
        obj_points.at<float>(i, 0) -= center_x;
        obj_points.at<float>(i, 1) -= center_y;
        obj_points.at<float>(i, 2) -= center_z;
    }
}


String string_pose(Pose pose)
{
    String str;
    // Strin
    str += "x: " + to_string(pose.pose.x);
    str += " y: " + to_string(pose.pose.y);
    str += " z: " + to_string(pose.pose.z);

    str += " x: " + to_string(pose.rotation.x);
    str += " y: " + to_string(pose.rotation.y);
    str += " z: " + to_string(pose.rotation.z);
    return str;
}

Point3d rotationMatrixToEAngles(Mat R)
{
    double sy = sqrtf(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2f(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2f(-R.at<double>(2, 0), sy);
        z = atan2f(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else
    {
        x = atan2f(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2f(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return Point3d(x, y, z);
}
// cv::rot
Mat R_flip_gen(double x, double y, double z)
{
    Mat _R_flip = Mat(3, 3, CV_64F, cvScalar(0.));
    _R_flip.at<double>(0, 0) = x;
    _R_flip.at<double>(1, 1) = y;
    _R_flip.at<double>(2, 2) = z;
    return _R_flip;
}

} // namespace Markers
// void Solver{
// }

