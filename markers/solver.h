#ifndef SOLVER_H_
#define SOLVER_H_

#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
// #include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include <opencv2/aruco.hpp>
#include "math.h"
#include "markers.h"
#include "string.h"

using namespace cv;
// using namespace markers;
namespace markers
{
class Solver
{
public:
    Solver();
    // Solver(Mat r_flip);
    void set_camera_conf(Mat cameraMatrix, Mat distCoeffs);
    bool solve(Mat objPoints, Mat imgPoints, Pose &pose, bool useExtrinsicGuess = false);
    bool solve(Mat objPoints, Mat imgPoints, Pose &pose, Mat &image, bool useExtrinsicGuess = false);
    void load_camera_conf(String path);

private:
    Mat _cameraMatrix;
    Mat _distCoeffs;
    Mat _R_flip;
};
} // namespace Markers
#endif
