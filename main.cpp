#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
// #include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include <opencv2/aruco.hpp>
#include "math.h"
#include "markers/markers.h"
#include "markers/solver.h"
#include "markers/aruco_markers.h"
// #include "markers/solver.cpp"

using namespace std;
using namespace cv;
using namespace Markers;


/*
    DICT_4X4_50     0
    DICT_4X4_100    1
    DICT_4X4_250    2
    DICT_4X4_1000   3
    DICT_5X5_50     4
    DICT_5X5_100    5
    DICT_5X5_250    6
    DICT_5X5_1000   7
    DICT_6X6_50     8
    DICT_6X6_100    9
    DICT_6X6_250    10
    DICT_6X6_1000   11
    DICT_7X7_50     12
    DICT_7X7_100    13
    DICT_7X7_250    14
    DICT_7X7_1000   15
    DICT_ARUCO_ORIGINAL
 */

int main(){
    // ArucoMarsol
    FileStorage fs2("config.yml", FileStorage::READ);
    // FileStorage fs3();
    string map = "./map.txt";
    fs2["map"] >> map;
    string calibration_file = "./log.yml";
    fs2["calibration_file"] >> calibration_file;
    string map_jpeg = "./map.jpg";
    fs2["map_jpeg"] >> map_jpeg;
    int map_jpeg_size = 1000;
    fs2["map_jpeg_size"] >> map_jpeg_size;
    int dictinary = 3;
    fs2["dictinary"] >> dictinary;
    int cam_id = 2;
    fs2["cam_id"] >> cam_id;
    
    fs2.release();

    Solver solver;
    solver.load_camera_conf("./log.yml");

    ArucoMarkersDetector aruco_detector(cv::aruco::getPredefinedDictionary(dictinary));
    aruco_detector.loadMap(map);
    // aruco_detector.addMarker(0, 0.05, Point3f(0, 0, 0));
    // aruco_detector.addMarker(40, 0.05, Point3f(0.10, 0, 0));
    // aruco_detector.addMarker(78, 0.05, Point3f(0, 0.1, 0));
    // aruco_detector.addMarker(70, 0.025, Point3f(0.05, 0.05, 0));
    // aruco_detector.addMarker(99, 0.05, Point3f(0.1, 0.1, 0));

    aruco_detector.genBoard();
    Mat map_img = aruco_detector.drawBoard(cv::Size(map_jpeg_size, map_jpeg_size));
    imwrite(map_jpeg, map_img);
    // Mat map_img;/
    // cv::aruco::drawPlanarBoard(aruco_detector._board, cv::Size(1000, 1000), map_img);
    // imshow("marker", map_img);
    // Mat viz;
    // map_img.copyTo(viz);
    

    // Mat objPoints, imgPoints;
    // aruco_detector.detect(viz, objPoints, imgPoints, viz);
    // // aruco_detector.drawViz(viz);
    // imshow("viz", viz);
    // // cout << objPoints << "img: " << imgPoints << "\n";
    // Pose pose;
    // if (solver.solve(objPoints, imgPoints, pose)){
    //     // std::cout << "Markers not found" << endl;
    //     std::cout << string_pose(pose) << endl;
    // }
    // else {
    //     std::cout << "Markers not found" << endl;
    // }
    
    // cin >> cam_id;
    cv::VideoCapture cap(cam_id);
    while (waitKey(4) != 'q'){
        Mat frame;
        cap >> frame;
        // cout << "frame w: " << frame.rows << " frame h: " << frame.cols << "\n";
        Mat viz;
        frame.copyTo(viz);
        

        Mat objPoints, imgPoints;
        aruco_detector.detect(frame, objPoints, imgPoints, viz);
        // aruco_detector.drawViz(viz);
        
        // cout << objPoints << "img: " << imgPoints << "\n";
        Pose pose;
        if (solver.solve(objPoints, imgPoints, pose, viz)){
            // std::cout << "Markers not found" << endl;
            std::cout << string_pose(pose) << endl;
        }
        else {
            std::cout << "Markers not found" << endl;
        }
        imshow("viz", viz);

    }
    cap.release();
    //waitKey(0);
      
    // aruco_detector.genBoard();

    // aruco_detector.drawMarkers();
    // solver.solve()
    return 0;
}
