#ifndef ARUCOMARKERS_H_
#define ARUCOMARKERS_H_

#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
// #include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include <opencv2/aruco.hpp>
#include "math.h"
#include "markers.h"
// class Marker{
// };
// struct Marker {
//     int id;
//     std::vector<cv::Point2f> points;
// };
using namespace cv;
using namespace cv::aruco;
// using namespace markers;

namespace markers
{
void _drawPlanarBoard(Board *_board, Size outSize, OutputArray _img, int marginSize,
					  int borderBits);

struct ArucoMarkerMap
{
    int id;
    float size;
    Point3f rotation;
    Point3f point;
};
// struct ArucoMarkerMapPoints{

// }

class ArucoMarkersDetector
{
public:
    ArucoMarkersDetector();
    ArucoMarkersDetector(cv::Ptr<cv::aruco::Dictionary> dictionary);

    void setDictionary(cv::Ptr<cv::aruco::Dictionary> dictionary);
    void addMarker(int id, float size, Point3f point, Point3f rotation);
    void genBoard();
    // void drawMarkers();
    Mat drawBoard(cv::Size size);
    Mat drawMarker(int ids);
    // void drawViz(Mat viz);

    bool detect(cv::Mat image, cv::Mat &objPoints, cv::Mat &imgPoints);
    bool detect(cv::Mat image, cv::Mat &objPoints, cv::Mat &imgPoints, cv::Mat &outImage);
    void loadMap(std::string);

private:
    std::vector<int> _ids_to_detect;
    std::vector<ArucoMarkerMap> _markers_map;
    cv::Ptr<cv::aruco::Board> _board;
    cv::Ptr<cv::aruco::Dictionary> _dictionary;
    std::vector<std::vector<cv::Point2f>> _corners;
    void _getBoardObjectAndImagePoints(const Ptr<aruco::Board> &board, InputArrayOfArrays detectedCorners,
                                       InputArray detectedIds, OutputArray objPoints, OutputArray imgPoints);
};
} // namespace Markers
#endif
