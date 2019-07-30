//by Vasily Yuryev 2019
#include <chrono>
#include <cmath>
#include <iostream>
#include <sys/time.h>
#include <thread>
#include <time.h>
// #include <iostream>
#include "opencv2/opencv.hpp"

#include <vector>
// #include "opencv2/imgproc.hpp"
#include "markers/aruco_markers.h"
#include "markers/markers.h"
#include "markers/solver.h"
#include "math.h"
#include "opencv2/ximgproc.hpp"
#include <opencv2/aruco.hpp>

#include <mavsdk/mavsdk.h>
// #include <mavsdk/plugins/>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using namespace cv;
using namespace markers;

using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

#define ERROR_CONSOLE_TEXT "\033[31m"     // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m"     // Restore normal console colour



// Handles Action's result
inline void action_error_exit(Action::Result result,
                              const std::string &message)
{
    if (result != Action::Result::SUCCESS)
    {
        std::cerr << ERROR_CONSOLE_TEXT << message << Action::result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles Offboard's result
inline void offboard_error_exit(Offboard::Result result,
                                const std::string &message)
{
    if (result != Offboard::Result::SUCCESS)
    {
        std::cerr << ERROR_CONSOLE_TEXT << message << Offboard::result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles connection result
inline void connection_error_exit(ConnectionResult result,
                                  const std::string &message)
{
    if (result != ConnectionResult::SUCCESS)
    {
        std::cerr << ERROR_CONSOLE_TEXT << message << connection_result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Logs during Offboard control
inline void offboard_log(const std::string &offb_mode, const std::string msg)
{
    std::cout << "[" << offb_mode << "] " << msg << std::endl;
}

uint64_t get_time_usec()
{
    static struct timeval _time_stamp;
    gettimeofday(&_time_stamp, NULL);
    return _time_stamp.tv_sec * 1000000 + _time_stamp.tv_usec;
}

// void imageCb(const sensor_msgs::ImageConstPtr &msg);

// ROS
class NormalVersion
{

    Mat cameraMatrix, distCoeffs;

    Mavsdk dc;
    ConnectionResult connection_result;
    // System system(dc, 0, 0);
    std::shared_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough;
    std::shared_ptr<mavsdk::Action> action;
    std::shared_ptr<mavsdk::Offboard> offboard;
    std::shared_ptr<mavsdk::Telemetry> telemetry;

    //Config
    std::string connection_url = "url://localhost:5785";
    string calibration_file = "./log.yml";
    string map = "./map.txt";
    string map_jpeg = "./map.jpg";
    int map_jpeg_size = 1000;
    int dictinary = 3;
    int cam_id = 2;

    Solver solver;
    ArucoMarkersDetector aruco_detector;

public:
    void usage(std::string bin_name)
    {
        std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name
                  << " <connection_url>" << std::endl
                  << "Connection URL format should be :" << std::endl
                  << " For TCP : tcp://[server_host][:server_port]" << std::endl
                  << " For UDP : udp://[bind_host][:bind_port]" << std::endl
                  << " For Serial : serial:///path/to/serial/dev[:baudrate]"
                  << std::endl
                  << "For example, to connect to the simulator use URL: udp://:14540"
                  << std::endl;
    }
    bool process_vison_vpe(Mat frame, Mat &viz)
    {
        frame.copyTo(viz);
        // std::share

        Mat objPoints, imgPoints;
        aruco_detector.detect(frame, objPoints, imgPoints, viz);
        // aruco_detector.drawViz(viz);

        // cout << objPoints << "img: " << imgPoints << "\n";
        Pose pose;
        if (solver.solve(objPoints, imgPoints, pose, viz))
        {
            // std::cout << "Markers not found"<< endl;
            // std::cout << string_pose(pose) << std::endl;
            mavlink_message_t message_vpe;
            mavlink_vision_position_estimate_t vpe;

            vpe.x = pose.pose.x;
            vpe.y = pose.pose.y;
            vpe.z = pose.pose.z;
            vpe.roll = pose.rotation.y;
            vpe.pitch = pose.rotation.x;
            vpe.yaw = pose.rotation.z;
            // vpe.usec =
            if (not vpe.usec)
                vpe.usec = (uint32_t)(get_time_usec() / 1000);

            // mavlink_message_t message;
            mavlink_msg_vision_position_estimate_encode(
                mavlink_passthrough->get_target_sysid(),
                mavlink_passthrough->get_target_compid(), &message_vpe, &vpe);

            mavlink_passthrough->send_message(message_vpe);
            return true;
        }
        else
        {
            return false;
        }
    }
    void load_config()
    {
        FileStorage fs2("config.yml", FileStorage::READ);
        fs2["map"] >> map;
        fs2["calibration_file"] >> calibration_file;

        fs2["connection_url"] >> connection_url;

        fs2["map_jpeg"] >> map_jpeg;
        fs2["map_jpeg_size"] >> map_jpeg_size;
        fs2["dictinary"] >> dictinary;
        fs2["cam_id"] >> cam_id;
        fs2.release();
    }
    void init_marker_reg()
    {

        solver.load_camera_conf(calibration_file);

        // solver.set_camera_conf(cameraMatrix, distCoeffs);

        aruco_detector.setDictionary(cv::aruco::getPredefinedDictionary(dictinary));

        aruco_detector.loadMap(map);
        aruco_detector.genBoard();
        Mat map_img =
            aruco_detector.drawBoard(cv::Size(map_jpeg_size, map_jpeg_size));
        imwrite(map_jpeg, map_img);
    }
    void init_mavlink()
    {

        // connection_url = argv[1];
        connection_result = dc.add_any_connection(connection_url);

        if (connection_result != ConnectionResult::SUCCESS)
        {
            std::cout << ERROR_CONSOLE_TEXT << "Connection failed: "
                      << connection_result_str(connection_result) << NORMAL_CONSOLE_TEXT
                      << std::endl;
            // return 1;
        }

        // Wait for the system to connect via heartbeat
        while (!dc.is_connected())
        {
            std::cout << "Wait for system to connect via heartbeat" << std::endl;
            sleep_for(seconds(1));
        }
        // system = mavsdk::System::System(&dc.system());
        // System got discovered.
        // system& = &dc.system();
        // System(
        // system = new System::System(&dc.system());
        // System::System()
        System &system = dc.system();
        action = std::make_shared<Action>(system);
        offboard = std::make_shared<Offboard>(system);
        telemetry = std::make_shared<Telemetry>(system);
        // mavsdk::MavlinkPassthrough::MavlinkPassthrough(system)
        mavlink_passthrough = std::make_shared<MavlinkPassthrough>(system);
    }

    void use_camera()
    {
        cv::VideoCapture cap(cam_id);
        std::cout << "System is ready" << std::endl;

        // Main loop
        while (waitKey(1) != 'q')
        {
            Mat frame;
            cap >> frame;
            // cout << "frame w: " << frame.rows << " frame h: " << frame.cols << "\n";
            Mat viz;
            process_vison_vpe(frame, viz);
            // else {
            //   // std::cout << "no markers\n";
            //   sleep_for(milliseconds(20));
            // }
            std::cout << telemetry->position_velocity_ned() << std::endl;
            imshow("vizz", viz);
            sleep_for(milliseconds(20));
        }

        sleep_for(seconds(3));
        std::cout << "Finished..." << std::endl;
    }
    NormalVersion()
    {
        load_config();
        init_marker_reg();
        init_mavlink();
        use_camera();
    }
    ~NormalVersion()
    {
    }
};

int main()
{

    // ros::init(argc, argv, "sitl_ver_node");
    NormalVersion an;
    // ros::spin();
    return 0;
    // return EXIT_SUCCESS;
}
