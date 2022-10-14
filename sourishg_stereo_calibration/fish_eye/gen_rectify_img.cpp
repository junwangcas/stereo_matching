#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "popt_pp.h"
// reference: https://sourishghosh.com/2016/stereo-calibration-cpp-opencv/

using namespace std;
using namespace cv;

void PrintMat(const cv::Mat &mat, const std::string &name)
{
    std::cout << name << " " << mat << "\n";
}

void PrintMat(const cv::InputArray &mat, const std::string &name)
{
    std::cout << name << " " << mat.getMat() << "\n";
}

void ShowHconImgs(cv::Mat &img1, cv::Mat &img2, std::string window_name)
{
    bool show_img = false;
    if (!show_img) {
        return;
    }
    cv::Mat left_img_rotate, right_img_rotate, left_right_rect_show;
    cv::rotate(img1, left_img_rotate, cv::ROTATE_90_CLOCKWISE);
    cv::rotate(img2, right_img_rotate, cv::ROTATE_90_CLOCKWISE);
    cv::hconcat(left_img_rotate, right_img_rotate, left_right_rect_show);
    cv::imshow(window_name, left_right_rect_show);
}

int main(int argc, char const *argv[])
{
    char* leftimg_filename;
    char* rightimg_filename;
    char* rectify_file;
    char* left_rect_rot_filename;
    char* right_rect_rot_filename;
    char* left_right_hcat_filename;

    static struct poptOption options[] = {
            { "leftimg_filename",'l',POPT_ARG_STRING,&leftimg_filename,0,"Left imgage path","STR" },
            { "rightimg_filename",'r',POPT_ARG_STRING,&rightimg_filename,0,"Right image path","STR" },
            { "rectify_file",'c',POPT_ARG_STRING,&rectify_file,0,"rectify_file","STR" },
            { "left_rect_rot_filename",'L',POPT_ARG_STRING,&left_rect_rot_filename,0,"left_rect_rot_filename","STR" },
            { "right_rect_rot_filename",'R',POPT_ARG_STRING,&right_rect_rot_filename,0,"right_rect_rot_filename","STR" },
            { "left_right_hcat_filename",'H',POPT_ARG_STRING,&left_right_hcat_filename,0,"left_right_hcat_filename","STR" },
            POPT_AUTOHELP
            { NULL, 0, 0, NULL, 0, NULL, NULL }
    };

    POpt popt(NULL, argc, argv, options, 0);
    int c;
    while((c = popt.getNextOpt()) >= 0) {}

    Mat R1, R2, P1, P2, Q;
    Mat K1, K2, R;
    Vec3d T;
    Mat D1, D2;
    Mat img1 = imread(leftimg_filename, CV_LOAD_IMAGE_GRAYSCALE);
    Mat img2 = imread(rightimg_filename, CV_LOAD_IMAGE_GRAYSCALE);

    cv::FileStorage fs1(rectify_file, cv::FileStorage::READ);
    fs1["K1"] >> K1;
    fs1["K2"] >> K2;
    fs1["D1"] >> D1;
    fs1["D2"] >> D2;
    fs1["R"] >> R;
    fs1["T"] >> T;
    PrintMat(K1, "K1");
    PrintMat(K2, "K2");
    PrintMat(D1, "D1");
    PrintMat(D2, "D2");
    PrintMat(R, "R");
    std::cout << "T: " << T << "\n";

    fs1["R1"] >> R1;
    fs1["P1"] >> P1;
    fs1["R2"] >> R2;
    fs1["P2"] >> P2;
    fs1["Q"] >> Q;

    cv::Mat lmapx, lmapy, rmapx, rmapy;
    cv::Mat imgU1, imgU2;

    // K1 - 内参， D1 - 畸变参数, R1 - 基本是一个单位矩阵，P1 - 貌似是一个K阵和T
    cv::fisheye::initUndistortRectifyMap(K1, D1, R1, P1, img1.size(), CV_32F, lmapx, lmapy);
    cv::fisheye::initUndistortRectifyMap(K2, D2, R2, P2, img2.size(), CV_32F, rmapx, rmapy);
    cv::remap(img1, imgU1, lmapx, lmapy, cv::INTER_CUBIC);
    cv::remap(img2, imgU2, rmapx, rmapy, cv::INTER_CUBIC);


    cv::Mat left_img_rotate, right_img_rotate, left_right_rect_show;
    cv::rotate(imgU1, left_img_rotate, cv::ROTATE_90_CLOCKWISE);
    cv::rotate(imgU2, right_img_rotate, cv::ROTATE_90_CLOCKWISE);

    {
        // visualization
        ShowHconImgs(imgU1, imgU2, "left-right-rectify");
        ShowHconImgs(img1, img2, "raw images");
    }

    {
        // save
        imwrite(left_rect_rot_filename, left_img_rotate);
        imwrite(right_rect_rot_filename, right_img_rotate);
        cv::Mat lr_img_rotate;
        cv::hconcat(left_img_rotate, right_img_rotate, lr_img_rotate);
        imwrite(left_right_hcat_filename, lr_img_rotate);
    }
    return 0;
}