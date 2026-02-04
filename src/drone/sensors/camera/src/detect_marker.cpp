#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include <cmath>

using namespace cv;
using namespace std;

// Adjust based on length of aruco
const float ARUCO_MARKER_SIZE = 0.1 // 0.1 meters

int main(int argc, char **argv){
    VideoCapture cap(0, CAP_V4L2);
    cap.set(CAP_PROP_FRAME_HEIGHT, 1080);
    cap.set(CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(CAP_PROP_FPS, 30);

    if(cap.isOpenned()){
        cerr << "Couldn't open camera" << endl;
        return -1;
    }

    Mat cameraMatrix = Mat(3, 3, CV_32F, camera_matrix_data);
    Mat distCoeffs = Mat(1, 5, CV_32F, dist_coeffs_data);

    aruco::DetectorParameters detectorParams = aruco::DetectorParameters();
    
    // LOWER TO DETECT MARKERS THAT ARE SMALLER OR FARTHER AWAY
    detectorParams.minMarkerPerimeterRate = 0.01; 
    
    // Polish corners
    detectorParams.cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; 
    
    // Adaptive thresholding window size
    detectorParams.adaptiveThreshWinSizeMin = 3;
    detectorParams.adaptiveThreshWinSizeMax = 23;
    detectorParams.adaptiveThreshWinSizeStep = 10;


    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

    aruco::ArucoDetector detector(dictionary, detectorParams);

    // Define the 3D coordinates of the marker corners
    vector<Point3f> objPoints;
    float halfSize = MARKER_SIZE / 2.0f;
    objPoints.push_back(Point3f(-halfSize, halfSize, 0));
    objPoints.push_back(Point3f(halfSize, halfSize, 0));
    objPoints.push_back(Point3f(halfSize, -halfSize, 0));
    objPoints.push_back(Point3f(-halfSize, -halfSize, 0));

    Mat frame, gray;
    vector<int> ids;
    vector<vector<Point2f>> corners, rejected;

    cout << "Start Detection" << endl;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // Convert to grayscale
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Detect
        detector.detectMarkers(gray, corners, ids, rejected);

        if (ids.size() > 0) {
            aruco::drawDetectedMarkers(frame, corners, ids);

            for (size_t i = 0; i < ids.size(); i++) {
                Vec3d rvec, tvec;
                
                // Pose Estimation
            }
        }

        imshow("Drone's View", frame);
        if (waitKey(1) == 27) break; // Escape key
    }

    return 0;
}