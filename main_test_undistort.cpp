
// reference: https://stackoverflow.com/questions/64903037/undistort-a-fisheye-image-using-cvfisheyeundistortimage
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream>

using namespace std;
using namespace cv;

void GetCam0Param(cv::Mat &cameraMatrix, cv::Mat &distortionCoeffs)
{
    cameraMatrix.at<double>(0, 0) = 236.6843023; // fx
    cameraMatrix.at<double>(0, 1) = 0;
    cameraMatrix.at<double>(0, 2) = 321.4439661; // cx
    cameraMatrix.at<double>(1, 0) = 0;
    cameraMatrix.at<double>(1, 1) = 236.3530385; // fy
    cameraMatrix.at<double>(1, 2) = 237.7165396; // cy
    cameraMatrix.at<double>(2, 0) = 0;
    cameraMatrix.at<double>(2, 1) = 0;
    cameraMatrix.at<double>(2, 2) = 1;

    distortionCoeffs.at<double>(0,0) = 0.208638285;
    distortionCoeffs.at<double>(1,0) = -0.169312622;
    distortionCoeffs.at<double>(2,0) = 0.06429176061;
    distortionCoeffs.at<double>(3,0) = -0.01059171675;
}

void GetCam3Param(cv::Mat &cameraMatrix, cv::Mat &distortionCoeffs)
{
    cameraMatrix.at<double>(0, 0) = 235.6928476; // fx
    cameraMatrix.at<double>(0, 1) = 0;
    cameraMatrix.at<double>(0, 2) = 322.3648277; // cx
    cameraMatrix.at<double>(1, 0) = 0;
    cameraMatrix.at<double>(1, 1) = 235.6722098; // fy
    cameraMatrix.at<double>(1, 2) = 238.0558978; // cy
    cameraMatrix.at<double>(2, 0) = 0;
    cameraMatrix.at<double>(2, 1) = 0;
    cameraMatrix.at<double>(2, 2) = 1;

    distortionCoeffs.at<double>(0,0) = 0.2140582412;
    distortionCoeffs.at<double>(1,0) = -0.1750094932;
    distortionCoeffs.at<double>(2,0) = 0.06449137836;
    distortionCoeffs.at<double>(3,0) = -0.009939820053;
}

bool ParseArgv(int argc, char *argv[], std::string &input_file, std::string &output_file, int &cam_id)
{
    std::string help_str = "Usage ./main -i input_file.bmp -o output_file.bmp -cam_id 0\n";
    if (argc == 1) {
        std::string path = "/sdcard/stereo_matching/testing/testcase1/";
        input_file = path + "cam0_571667480915.bmp";
        output_file = path + "cam0_rectify.bmp";
        return true;
    } else if (argc == 7) {
        if (strcmp(argv[1], "-i") != 0 || strcmp(argv[3], "-o") != 0 || strcmp(argv[5], "-cam_id")  != 0) {
            std::cout << help_str;
            return false;
        }
        input_file = argv[2];
        output_file = argv[4];
        cam_id = stoi(argv[6]);
        return true;
    } else {
        std::cout << help_str;
        return false;
    }
}

int main(int argc, char *argv[]) {
//    std::cout << "argc " << argc << " argv " <<  "\n";
//    for (int i = 0; i < argc; i++) {
//        std::cout << argv[i] << ", ";
//    }
//    std::cout << "\n";

    std::string input_file;
    std::string output_file;
    int cam_id;
    if (!ParseArgv(argc, argv, input_file, output_file, cam_id)) {
        return 0;
    }

    cv::Mat cameraMatrix = cv::Mat(3,3, cv::DataType<double>::type);
    cv::Mat distortionCoeffs = cv::Mat(4,1, cv::DataType<double>::type);

    if (cam_id == 0) {
        GetCam0Param(cameraMatrix, distortionCoeffs);
    } else if (cam_id == 3) {
        GetCam3Param(cameraMatrix, distortionCoeffs);
    }

    cv::Mat E = cv::Mat::eye(3, 3, cv::DataType<double>::type);

    cv::Mat input_frame = cv::imread(input_file);

    cv::Size size = { input_frame.cols, input_frame.rows };

    cv::Mat map1;
    cv::Mat map2;
    //cv::fisheye::undistortImage(input_frame,output_frame,cameraMatrix,distortionCoeffs, E, cv::Size(input_frame.cols,input_frame.rows));

    cv::fisheye::initUndistortRectifyMap(cameraMatrix, distortionCoeffs, E, cameraMatrix, size, CV_16SC2, map1, map2);

    cv::Mat undistort;

    cv::remap(input_frame, undistort, map1, map2, cv::INTER_LINEAR,
              CV_HAL_BORDER_CONSTANT);

    cv::imshow("Input Image", input_frame);
    cv::imshow("Output Image", undistort);
    cv::waitKey(-1);
    cv::imwrite(output_file, undistort);

    return 0;
}
