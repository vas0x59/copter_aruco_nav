// #include <iostream>
// #include <vector>
// #include "opencv2/opencv.hpp"
// // #include "opencv2/imgproc.hpp"
// #include "opencv2/ximgproc.hpp"
// #include <opencv2/aruco.hpp>
// #include "math.h"
// #include "markers.h"
#include "aruco_markers.h"
// class Marker{
// };
// struct Marker {
//     int id;
//     std::vector<cv::Point2f> points;
// };
using namespace cv;
using namespace std;
using namespace cv::aruco;
using namespace markers;

// namespace markers
// {

void _drawPlanarBoard(Board *_board, Size outSize, OutputArray _img, int marginSize,
					  int borderBits)
{

	CV_Assert(outSize.area() > 0);
	CV_Assert(marginSize >= 0);

	_img.create(outSize, CV_8UC1);
	Mat out = _img.getMat();
	out.setTo(Scalar::all(255));
	out.adjustROI(-marginSize, -marginSize, -marginSize, -marginSize);

	// calculate max and min values in XY plane
	CV_Assert(_board->objPoints.size() > 0);
	float minX, maxX, minY, maxY;
	minX = maxX = _board->objPoints[0][0].x;
	minY = maxY = _board->objPoints[0][0].y;

	for (unsigned int i = 0; i < _board->objPoints.size(); i++)
	{
		for (int j = 0; j < 4; j++)
		{
			minX = min(minX, _board->objPoints[i][j].x);
			maxX = max(maxX, _board->objPoints[i][j].x);
			minY = min(minY, _board->objPoints[i][j].y);
			maxY = max(maxY, _board->objPoints[i][j].y);
		}
	}

	float sizeX = maxX - minX;
	float sizeY = maxY - minY;

	// proportion transformations
	float xReduction = sizeX / float(out.cols);
	float yReduction = sizeY / float(out.rows);

	// determine the zone where the markers are placed
	if (xReduction > yReduction)
	{
		int nRows = int(sizeY / xReduction);
		int rowsMargins = (out.rows - nRows) / 2;
		out.adjustROI(-rowsMargins, -rowsMargins, 0, 0);
	}
	else
	{
		int nCols = int(sizeX / yReduction);
		int colsMargins = (out.cols - nCols) / 2;
		out.adjustROI(0, 0, -colsMargins, -colsMargins);
	}

	// now paint each marker
	Dictionary &dictionary = *(_board->dictionary);
	Mat marker;
	Point2f outCorners[3];
	Point2f inCorners[3];
	for (unsigned int m = 0; m < _board->objPoints.size(); m++)
	{
		// transform corners to markerZone coordinates
		for (int j = 0; j < 3; j++)
		{
			Point2f pf = Point2f(_board->objPoints[m][j].x, _board->objPoints[m][j].y);
			// move top left to 0, 0
			pf -= Point2f(minX, minY);
			pf.x = pf.x / sizeX * float(out.cols);
			pf.y = (1.0f - pf.y / sizeY) * float(out.rows);
			outCorners[j] = pf;
		}

		// get marker
		Size dst_sz(outCorners[2] - outCorners[0]); // assuming CCW order
		// dst_sz.width = dst_sz.height = std::min(dst_sz.width, dst_sz.height); //marker should be square
		double diag = std::round(std::hypot(dst_sz.width, dst_sz.height));
		int side = std::round(diag / std::sqrt(2));
		side = std::max(side, 10);

		dictionary.drawMarker(_board->ids[m], side, marker, borderBits);

		// interpolate tiny marker to marker position in markerZone
		inCorners[0] = Point2f(-0.5f, -0.5f);
		inCorners[1] = Point2f(marker.cols - 0.5f, -0.5f);
		inCorners[2] = Point2f(marker.cols - 0.5f, marker.rows - 0.5f);

		// remove perspective
		Mat transformation = getAffineTransform(inCorners, outCorners);
		warpAffine(marker, out, transformation, out.size(), INTER_LINEAR,
				   BORDER_TRANSPARENT);
	}
}

ArucoMarkersDetector::ArucoMarkersDetector(cv::Ptr<cv::aruco::Dictionary> dictionary)
{
	_dictionary = dictionary;
}

void ArucoMarkersDetector::loadMap(std::string filename)
{
	std::ifstream f(filename);
	std::string line;

	if (!f.good())
	{
		// PRINT_FATAL("aruco_map: %s - %s", strerror(errno), filename.c_str());
		// PRINT::shutdown();
		cout << "Bad FILE !!!!!";
	}

	while (std::getline(f, line))
	{
		int id;
		double length, x, y, z, yaw, pitch, roll;

		std::istringstream s(line);

		// Read first character to see whether it's a comment
		char first = 0;
		if (!(s >> first))
		{
			// No non-whitespace characters, must be a blank line
			continue;
		}

		if (first == '#')
		{
			cout << "aruco_map: Skipping line as a comment: " << line.c_str();
			continue;
		}
		else if (isdigit(first))
		{
			// Put the digit back into the stream
			// Note that this is a non-modifying putback, so this should work with istreams
			// (see https://en.cppreference.com/w/cpp/io/basic_istream/putback)
			s.putback(first);
		}
		else
		{
			// Probably garbage data; inform user and throw an exception, possibly killing nodelet
			cout << "aruco_map: Malformed input: " << line.c_str();
			// PRINT::shutdown();
			// throw std::runtime_error("Malformed input");
			continue;
		}

		if (!(s >> id >> length >> x >> y))
		{
			cout << "aruco_map: Not enough data in line:; "
					"Each marker must have at least id, length, x, y fields"
				 << line.c_str();
			continue;
		}
		// Be less strict about z, yaw, pitch roll
		if (!(s >> z))
		{
			cout << "aruco_map: No z coordinate provided for marker, assuming 0" << id;
			z = 0;
		}
		if (!(s >> yaw))
		{
			// PRINT_DEBUG("aruco_map: No yaw provided for marker %d, assuming 0", id);
			cout << "aruco_map: No yaw coordinate provided for marker, assuming 0" << id;
			yaw = 0;
		}
		if (!(s >> pitch))
		{
			cout << "aruco_map: No pitch coordinate provided for marker, assuming 0" << id;
			pitch = 0;
		}
		if (!(s >> roll))
		{
			cout << "aruco_map: No roll coordinate provided for marker, assuming 0" << id;
			roll = 0;
		}
		addMarker(id, length, Point3f(x, y, z), Point3f(pitch, roll, yaw));
	}

	// PRINT_INFO("aruco_map: loading %s complete (%d markers)", filename.c_str(), static_cast<int>(board_->ids.size()));
}

Mat ArucoMarkersDetector::drawMarker(int id)
{
	Mat img;
	cv::aruco::drawMarker(_dictionary, id, 0, img);
	return img;
}

Mat ArucoMarkersDetector::drawBoard(cv::Size size)
{
	Mat img;
	// cv::aruco
	// cv::aruco::dra
	_drawPlanarBoard(_board, size, img, 40, 1);
	// cv::aruco::drawMarker(dictionary, id, 0, img);
	return img;
}

void ArucoMarkersDetector::genBoard()
{
	std::vector<std::vector<Point3f>> objPoints;
	// objPoints.reserve(_markers_map.size());
	for (size_t i = 0; i < _markers_map.size(); i++)
	{
		std::vector<Point3f> marker_points;
		// marker_points.reserve(4);

		ArucoMarkerMap marker = _markers_map[i];
		float size = marker.size;
		float size_p = size / 2;

		// marker_points.push_back(Point3f(marker.point.x+size, marker.point.y+size, marker.point.z));
		// marker_points.push_back(Point3f(marker.point.x, marker.point.y+size, marker.point.z));
		// marker_points.push_back(Point3f(marker.point.x, marker.point.y, marker.point.z));
		// marker_points.push_back(Point3f(marker.point.x+size, marker.point.y, marker.point.z));

		vector<Point3f> corners;
		corners.resize(4);
		// corners[0] = marker.point;
		// corners[1] = corners[0] + Point3f(size, 0, 0);
		// corners[2] = corners[0] + Point3f(size, -size, 0);
		// corners[3] = corners[0] + Point3f(0, -size, 0);
		// cos

		// Point3f rot_v(sinf(marker.z_rotation)*size_p,  cosf(marker.z_rotation)*size_p, 0.0);
		//M_rotate = cv2.getRotationMatrix2D(Point2f(marker.point.x, marker.point.y
		//corners[0] = marker.point + Point3f(-size_p, size_p, 0);
		//corners[1] = marker.point + Point3f(size_p, size_p, 0);
		//corners[2] = marker.point + Point3f(size_p, -size_p, 0);
		//corners[3] = marker.point + Point3f(-size_p, -size_p, 0);

		corners[0] = marker.point + rotate3d(Point3f(-size_p, size_p, 0), Point3f(0, 0, 0), marker.rotation);
		corners[1] = marker.point + rotate3d(Point3f(size_p, size_p, 0), Point3f(0, 0, 0), marker.rotation);
		corners[2] = marker.point + rotate3d(Point3f(size_p, -size_p, 0), Point3f(0, 0, 0), marker.rotation);
		corners[3] = marker.point + rotate3d(Point3f(-size_p, -size_p, 0), Point3f(0, 0, 0), marker.rotation);

		// corners[0] = rotate3d(marker.point + Point3f(-size_p, size_p, 0), marker.point, marker.z_rotation);
		// corners[1] = rotate3d(marker.point + Point3f(size_p, size_p, 0), marker.point, marker.z_rotation);
		// corners[2] = rotate3d(marker.point + Point3f(size_p, -size_p, 0), marker.point, marker.z_rotation);
		// corners[3] = rotate3d(marker.point + Point3f(-size_p, -size_p, 0), marker.point, marker.z_rotation);

		// marker_points.push_back(Point3f(marker.point.x-size_p, marker.point.y+size_p, marker.point.z));
		// marker_points.push_back(Point3f(marker.point.x+size_p, marker.point.y+size_p, marker.point.z));
		// marker_points.push_back(Point3f(marker.point.x+size_p, marker.point.y-size_p, marker.point.z));
		// marker_points.push_back(Point3f(marker.point.x-size_p, marker.point.y-size_p, marker.point.z));

		objPoints.push_back(corners);
	}

	_board = cv::aruco::Board::create(objPoints, _dictionary, _ids_to_detect);
}
void ArucoMarkersDetector::addMarker(int id, float size, Point3f point, Point3f rotation)
{

	_ids_to_detect.push_back(id);

	ArucoMarkerMap marker;

	marker.id = id;
	marker.point = point;
	marker.size = size;
	marker.rotation = rotation;

	_markers_map.push_back(marker);
}

// void drawViz(Mat viz){
//     aruco::drawDetectedMarkers(viz, _corners, )
// }

bool ArucoMarkersDetector::detect(cv::Mat image, cv::Mat &objPoints, cv::Mat &imgPoints)
{
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> corners;
	cv::aruco::detectMarkers(image, _dictionary, corners, ids);
	_corners = corners;
	if (ids.size() > 0)
	{
		// CV_Assert(_corners.total() == _ids.total());

		// get object and image points for the solvePnP function
		// Mat /*objPoints, */ imgPoints;
		_getBoardObjectAndImagePoints(_board, corners, ids, objPoints, imgPoints);
		// double center_x = 0, center_y = 0, center_z = 0;
		// alignObjPointsToCenter(objPoints, center_x, center_y, center_z);
		// imgPoints.convertTo(imgPoints, CV_64F);
		return true;
	}
	else
	{
		return false;
	}
}

bool ArucoMarkersDetector::detect(cv::Mat image, cv::Mat &objPoints, cv::Mat &imgPoints, cv::Mat &outImage)
{
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> corners;
	cv::aruco::detectMarkers(image, _dictionary, corners, ids);
	_corners = corners;
	if (ids.size() > 0)
	{
		// CV_Assert(_corners.total() == _ids.total());

		// get object and image points for the solvePnP function
		// Mat /*objPoints, */ imgPoints;
		_getBoardObjectAndImagePoints(_board, corners, ids, objPoints, imgPoints);
		// double center_x = 0, center_y = 0, center_z = 0;
		// alignObjPointsToCenter(objPoints, center_x, center_y, center_z);

		// cv::aruco::get/
		// cv::aruco::getBoardObjectAndImagePoints(_board, corners, ids, objPoints, imgPoints);
		cv::aruco::drawDetectedMarkers(outImage, corners, ids);

		return true;
	}
	else
	{
		return false;
	}
}

void ArucoMarkersDetector::_getBoardObjectAndImagePoints(const Ptr<aruco::Board> &board, InputArrayOfArrays detectedCorners,
														 InputArray detectedIds, OutputArray objPoints, OutputArray imgPoints)
{

	CV_Assert(board->ids.size() == board->objPoints.size());
	CV_Assert(detectedIds.total() == detectedCorners.total());

	size_t nDetectedMarkers = detectedIds.total();

	vector<Point3f> objPnts;
	objPnts.reserve(nDetectedMarkers);

	vector<Point2f> imgPnts;
	imgPnts.reserve(nDetectedMarkers);

	// look for detected markers that belong to the board and get their information
	for (unsigned int i = 0; i < nDetectedMarkers; i++)
	{
		int currentId = detectedIds.getMat().ptr<int>(0)[i];
		for (unsigned int j = 0; j < board->ids.size(); j++)
		{
			if (currentId == board->ids[j])
			{
				for (int p = 0; p < 4; p++)
				{
					objPnts.push_back(board->objPoints[j][p]);
					imgPnts.push_back(detectedCorners.getMat(i).ptr<Point2f>(0)[p]);
				}
			}
		}
	}

	// create output
	Mat(objPnts).copyTo(objPoints);
	Mat(imgPnts).copyTo(imgPoints);
}
// } // namespace Markers

// void ArucoMarkersDetector::
