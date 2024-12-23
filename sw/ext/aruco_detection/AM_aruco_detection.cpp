#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <chrono>
#include <thread>

// Parameters
const float MARKER_SIZE = 0.035; 
const int pixel_w = 1280;
const int pixel_h = 720;
const std::string pathLoad = "/home/orangepi/paparazzi/sw/ext/aruco_detection/cameraCalibration_mapir_1440p.xml";
const std::string usb_num = "/dev/video0";

// Verbose Flags
bool verbose_detection = false;
bool verbose_fps = true;
bool verbose_debug = false;

// Load Camera Calibration
void loadCameraParameters(cv::Mat &cameraMatrix, cv::Mat &distCoeffs) {
    cv::FileStorage fs(pathLoad, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to load calibration file!" << std::endl;
        exit(EXIT_FAILURE);
    }
    fs["cM"] >> cameraMatrix;
    fs["dist"] >> distCoeffs;
    if (verbose_debug) {
        std::cout << "[DEBUG] Camera calibration loaded successfully." << std::endl;
    }
}

// Main Detection Loop
void arucoDetectionLoop() {
    cv::VideoCapture cap(usb_num, cv::CAP_V4L2);  // Force V4L2 to bypass GStreamer
    if (!cap.isOpened()) {
        std::cerr << "[ERROR] Failed to open camera!" << std::endl;
        return;
    }

    // Optimize camera settings
    cap.set(cv::CAP_PROP_FRAME_WIDTH, pixel_w);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, pixel_h);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_CONVERT_RGB, 1);  // Ensure RGB conversion to avoid strange sizes
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));  // Force YUYV format

    if (verbose_debug) {
        std::cout << "[DEBUG] Camera initialized with resolution: " 
                  << pixel_w << "x" << pixel_h << std::endl;
    }

    cv::Mat cameraMatrix, distCoeffs;
    loadCameraParameters(cameraMatrix, distCoeffs);

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    auto start_time = std::chrono::steady_clock::now();
    int frame_counter = 0;

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            if(verbose_debug){
                std::cerr << "[ERROR] Frame not captured!" << std::endl;
            }
            continue;
        }
        if(verbose_debug){
            std::cout << "[DEBUG] Captured frame size: " << frame.cols << "x" << frame.rows << std::endl;
        }
        

        // Optional: Re-enable resolution check if frame sizes stabilize
        if (frame.rows != pixel_h || frame.cols != pixel_w) {
            if (verbose_debug){
                std::cerr << "[ERROR] Frame size mismatch. Expected: " 
                        << pixel_w << "x" << pixel_h 
                        << ", Got: " << frame.cols << "x" << frame.rows << std::endl;
            }
            continue;
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;

        cv::aruco::detectMarkers(frame, dictionary, corners, ids, parameters);
        
        if (!ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, MARKER_SIZE, cameraMatrix, distCoeffs, rvecs, tvecs);
            for (size_t i = 0; i < ids.size(); i++) {
                if (verbose_detection) {
                    std::cout << "ID: " << ids[i] 
                              << " tvec: " << tvecs[i][0] << ", " 
                              << tvecs[i][1] << ", " << tvecs[i][2] << std::endl;
                }
            }
        }

        frame_counter++;
        auto current_time = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        
        if (elapsed_time >= 1.0) {
            double fps = frame_counter / elapsed_time;
            if (verbose_fps) {
                std::cout << "[INFO] FPS: " << fps << std::endl;
            }
            frame_counter = 0;
            start_time = std::chrono::steady_clock::now();
        }
    }
}

int main() {
    if (verbose_debug) {
        std::cout << "[DEBUG] Starting ArUco Detection..." << std::endl;
    }
    arucoDetectionLoop();
    return 0;
}
