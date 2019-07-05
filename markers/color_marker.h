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
//     std::vector<Point2f> points;
// };
using namespace cv;

#ifndef _COLORMARKERS_H_
#define _COLORMARKERS_H_


void detectColorMarkers(Mat image, std::vector<std::vector<cv::Point2f>> corners, std::vector<int> ids){
    
}

struct ColorMarkerMap
{
    int id;
    float size;
    Point3f point;
};
class ColorMarkerBoard {
    public:
    
    ColorMarkerBoard create(InputArrayOfArrays objPoints, InputArray ids);
    /// array of object points of all the marker corners in the board
    /// each marker include its 4 corners in CCW order. For M markers, the size is Mx4.
    std::vector< std::vector< Point3f > > objPoints;

    /// the dictionary of markers employed for this board
    // <Dictionary> dictionary;

    /// vector of the identifiers of the markers in the board (same size than objPoints)
    /// The identifiers refers to the board dictionary
    std::vector< int > ids;
}


// struct ArucoMarkerMapPoints{

// }

class ColorMarkersDetector
{
public:
    ColorMarkersDetector(Ptr<aruco::Dictionary> dictionary);
    void addMarker(int id, float size, Point3f point);
    void genBoard();
    // void drawMarkers();
    Mat drawBoard(Size size);
    Mat drawMarker(int id);
    void drawViz(Mat viz);
    
    bool detect(Mat image, Mat &objPoints, Mat &imgPoints);
    bool detect(Mat image, Mat &objPoints, Mat &imgPoints, Mat &outImage);
    void loadMap(std::string);
private:
    
    std::vector<int> _ids_to_detect;
    std::vector<ArucoMarkerMap> _markers_map;
    ColorMarkerBoard _board;
    // Ptr<aruco::Dictionary> _dictionary;
    std::vector<std::vector<Point2f>> _corners;
    void _getBoardObjectAndImagePoints(const Ptr<aruco::Board> &board, InputArrayOfArrays detectedCorners,
                                       InputArray detectedIds, OutputArray objPoints, OutputArray imgPoints);
    
    // std::vector<ArucoMark
    // std::vector<>
};
#endif