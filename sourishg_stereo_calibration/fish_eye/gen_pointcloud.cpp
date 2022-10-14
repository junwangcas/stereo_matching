#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "popt_pp.h"
#include "fstream"
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

void SaveToTxtFile(cv::Mat &out3d_img, std::string file_name) {
    fstream file_handle;
    file_handle.open(file_name, ios::out);

    auto img_size = out3d_img.size();
    for (uint32_t idrow = 0; idrow < img_size.height; idrow++) {
        for (uint32_t idcol = 0; idcol < img_size.width; idcol++) {
            float x = out3d_img.at<Vec3f>(idrow, idcol)[0];
            float y = out3d_img.at<Vec3f>(idrow, idcol)[1];
            float z = out3d_img.at<Vec3f>(idrow, idcol)[2];

            if (isinf(x) || isinf(y) || isinf(z)) {
                continue;
            }

            std::string xyzstr = std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + "\n";
            file_handle << xyzstr;
        }
    }
    file_handle.close();
}

int main(int argc, char const *argv[])
{
    char* disparity_filename;
    char* rectify_file;
    char* pointcloud_file;

    static struct poptOption options[] = {
            { "disparity_filename",'d',POPT_ARG_STRING,&disparity_filename,0,"disparity_filename","STR" },
            { "rectify_file",'c',POPT_ARG_STRING,&rectify_file,0,"rectify_file","STR" },
            { "pointcloud_file",'L',POPT_ARG_STRING,&pointcloud_file,0,"pointcloud_file","STR" },
            POPT_AUTOHELP
            { NULL, 0, 0, NULL, 0, NULL, NULL }
    };

    POpt popt(NULL, argc, argv, options, 0);
    int c;
    while((c = popt.getNextOpt()) >= 0) {}

    Mat Q;
    Mat disparity_img = imread(disparity_filename, CV_LOAD_IMAGE_ANYDEPTH);
    {
//        for (uint32_t idrow = 0; idrow < disparity_img.size().height; idrow++) {
//            for (uint32_t idcol = 0; idcol < disparity_img.size().width; idcol++) {
//                auto val = disparity_img.at<float>(idrow, idcol);
////                if (val > 0.001) {
//                    std::cout << val << ", ";
////                }
//            }
//        }
    }
//    std::cout << disparity_img.at<float>(10, 10) << "\n";
//    return 0;

    cv::FileStorage fs1(rectify_file, cv::FileStorage::READ);
    fs1["Q"] >> Q;
    PrintMat(Q, "Q");

    cv::Mat disparity_rot_img;
    cv::rotate(disparity_img, disparity_rot_img, cv::ROTATE_90_COUNTERCLOCKWISE);

    cv::Mat out3d_img;
    cv::reprojectImageTo3D(disparity_rot_img, out3d_img, Q, true);

    SaveToTxtFile(out3d_img, pointcloud_file);

    return 0;
}

