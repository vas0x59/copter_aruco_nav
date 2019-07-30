// #include <iostream>
// #include <vector>
// #include "opencv2/opencv.hpp"
// // #include "opencv2/imgproc.hpp"
// #include "opencv2/ximgproc.hpp"
// #include <opencv2/aruco.hpp>
// #include "math.h"
// #include "markers.h"
#include "solver.h"

using namespace cv;
using namespace std;
using namespace markers;
// namespace markers
// {
Solver::Solver()
{
    _R_flip = R_flip_gen(1, -1, -1);
}
// Solver::Solver(Mat r_flip) {
//     _R_flip = r_flip;
// }
void Solver::set_camera_conf(Mat cameraMatrix, Mat distCoeffs)
{
    _cameraMatrix = cameraMatrix;
    _distCoeffs = distCoeffs;
}
void Solver::load_camera_conf(String path)
{
    FileStorage fs2(path, FileStorage::READ);
    // FileStorage fs3();
    Mat cameraMatrix, distCoeffs;
    fs2["camera_matrix"] >> _cameraMatrix;
    fs2["distortion_coefficients"] >> _distCoeffs;
    // Mat(cameraMatrix).convertTo(_cameraMatrix, CV_64F);
    // Mat(distCoeffs).convertTo(_distCoeffs, CV_64F);

    fs2.release();
    // return false;
}

bool Solver::solve(Mat objPoints, Mat imgPoints, Pose &pose, bool useExtrinsicGuess)
{
    // std::cout << "cam mat: " << _cameraMatrix << " dist coeff" << _cameraMatrix << "\n";
    if (objPoints.total() == 0) // 0 of the detected markers in board
        return false;

    // std::cout << "objPoints: " << objPoints << std::endl;
    // std::cout  << "imgPoints: " << imgPoints << std::endl;
    // Mat _tvec;
    // Mat _rvec;
    cv::Vec3d _tvec, _rvec;

    solvePnP(objPoints, imgPoints, _cameraMatrix, _distCoeffs, _rvec, _tvec, useExtrinsicGuess);
    if (((int)objPoints.total() / 4) > 0)
    {
        // aruco::drawAxis(image, _cameraMatrix, _distCoeffs, _rvec,
        //       _tvec, 0.2);
        Mat pos_camera = Mat(3, 3, CV_64F, cvScalar(0.));
        Mat R_ct, R_tc;
        cv::Rodrigues(_rvec, R_ct);
        R_tc = R_ct.t();
        // double roll_marker, pitch_marker, yaw_marker;
        // Point3d marker_angles;
        pose.rotation = rotationMatrixToEAngles(_R_flip * R_tc);
        // roll_marker = marker_angles.x;
        // pitch_marker = marker_angles.y;
        // yaw_marker = marker_angles.z;
        // cout << "tvec" << _tvec << "\n";
        // cout << "marker_angles_raw  roll:" << roll_marker << " pitch:" << pitch_marker << " yaw:" << yaw_marker << endl;
        Mat tvec_mat = Mat(_tvec);
        R_tc = -R_tc;
        pos_camera = R_tc * tvec_mat;
        pose.pose.x = pos_camera.at<double>(0, 0);
        pose.pose.y = pos_camera.at<double>(0, 1);
        pose.pose.z = pos_camera.at<double>(0, 2);
        return true;
    }
    else
    {
        return false;
    }
    // divide by four since all the four corners are concatenated in the array for each marker
}
bool Solver::solve(Mat objPoints, Mat imgPoints, Pose &pose, Mat &image, bool useExtrinsicGuess)
{
    // std::cout << "cam mat: " << _cameraMatrix << " dist coeff" << _cameraMatrix << "\n";
    if (objPoints.total() == 0) // 0 of the detected markers in board
        return false;

    // std::cout << "objPoints: " << objPoints << std::endl;
    // std::cout  << "imgPoints: " << imgPoints << std::endl;
    // Mat _tvec;
    // Mat _rvec;
    cv::Vec3d _tvec, _rvec;

    solvePnP(objPoints, imgPoints, _cameraMatrix, _distCoeffs, _rvec, _tvec, useExtrinsicGuess);
    if (((int)objPoints.total() / 4) > 0)
    {
        aruco::drawAxis(image, _cameraMatrix, _distCoeffs, _rvec,
                        _tvec, 0.2);
        Mat pos_camera = Mat(3, 3, CV_64F, cvScalar(0.));
        Mat R_ct, R_tc;
        cv::Rodrigues(_rvec, R_ct);
        R_tc = R_ct.t();
        // double roll_marker, pitch_marker, yaw_marker;
        // Point3d marker_angles;
        pose.rotation = rotationMatrixToEAngles(_R_flip * R_tc);
        // roll_marker = marker_angles.x;
        // pitch_marker = marker_angles.y;
        // yaw_marker = marker_angles.z;
        // cout << "tvec" << _tvec << "\n";
        // cout << "marker_angles_raw  roll:" << roll_marker << " pitch:" << pitch_marker << " yaw:" << yaw_marker << endl;
        Mat tvec_mat = Mat(_tvec);
        R_tc = -R_tc;
        pos_camera = R_tc * tvec_mat;
        pose.pose.x = pos_camera.at<double>(0, 0);
        pose.pose.y = pos_camera.at<double>(0, 1);
        pose.pose.z = pos_camera.at<double>(0, 2);
        return true;
    }
    else
    {
        return false;
    }
    // divide by four since all the four corners are concatenated in the array for each marker
}
// } // namespace Markers