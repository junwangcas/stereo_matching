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

void SaveToTxtFile(cv::Mat &out3d_img, std::string file_name) {
    fstream file_handle;
    file_handle.open(file_name, ios::out);

    auto img_size = out3d_img.size();
    for (uint32_t idrow = 0; idrow < img_size.height; idrow++) {
        for (uint32_t idcol = 0; idcol < img_size.width; idcol++) {
            float x = out3d_img.at<Vec4f>(idrow, idcol)[0];
            float y = out3d_img.at<Vec4f>(idrow, idcol)[1];
            float z = out3d_img.at<Vec4f>(idrow, idcol)[2];
            int color = out3d_img.at<Vec4f>(idrow, idcol)[3];

            if (isinf(x) || isinf(y) || isinf(z) || z > 15 || z < -0.0) {
                continue;
            }

            std::string xyzstr = std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ", " + std::to_string(color) + "\n";
            file_handle << xyzstr;
        }
    }
    file_handle.close();
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
    float base_line = 0.11;
    char* disparity_filename;
    char* rectify_file;
    char* pointcloud_file;
    char* color_img_file;

    static struct poptOption options[] = {
            { "disparity_filename",'d',POPT_ARG_STRING,&disparity_filename,0,"disparity_filename","STR" },
            { "color_img_file",'i',POPT_ARG_STRING,&color_img_file,0,"color_img_file","STR" },
            { "rectify_file",'c',POPT_ARG_STRING,&rectify_file,0,"rectify_file","STR" },
            { "pointcloud_file",'L',POPT_ARG_STRING,&pointcloud_file,0,"pointcloud_file","STR" },
            POPT_AUTOHELP
            { NULL, 0, 0, NULL, 0, NULL, NULL }
    };

    POpt popt(NULL, argc, argv, options, 0);
    int c;
    while((c = popt.getNextOpt()) >= 0) {}

//    std::string file_name = "../../scripts/working_space/data/left_disp_float.tiff";
//    std::string file_out_pointcloud = "../../scripts/working_space/data/point_cloud.txt";
    std::string file_name = disparity_filename;
    std::string file_out_pointcloud = pointcloud_file;

    cv::Mat P1;
    cv::FileStorage fs1(rectify_file, cv::FileStorage::READ);
    fs1["P1"] >> P1;
    PrintMat(P1, "P1");

    double focal_length = P1.at<double>(0, 0); // 289 , 120
    double cx = P1.at<double>(0, 2); // 307.599660807135;
    double cy = P1.at<double>(1, 2); // 235.6465488645034;

    std::cout << "focal length : " << focal_length << " cx " << cx << " cy " << cy << " baseline " << base_line << "\n";


    cv::Mat left_disp_float = cv::imread(file_name, cv::IMREAD_ANYDEPTH);
    cv::Mat left_img = cv::imread(color_img_file, cv::IMREAD_ANYDEPTH);
//    cv::imshow("left_disp_float", left_disp_float);

    cv::Mat left_disp_rotbak, left_img_rotbak;
    cv::rotate(left_disp_float, left_disp_rotbak, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::rotate(left_img, left_img_rotbak, cv::ROTATE_90_COUNTERCLOCKWISE);
//    cv::imshow("left_disp_rotbak", left_disp_rotbak);

    double minVal;
    double maxVal;
    cv::minMaxLoc(left_disp_rotbak, &minVal, &maxVal);
    std::cout << "min " << minVal << " max " << maxVal << "\n";


    cv::Mat point_3d_img;
    point_3d_img.create(left_disp_rotbak.size(), CV_32FC4);

    double min_temp = 100;
    double max_temp = -100;
    for (uint32_t idrow = 0; idrow < left_disp_rotbak.size().height; idrow++) {
        for (uint32_t idcol = 0; idcol < left_disp_rotbak.size().width; idcol++) {
            float dispval = left_disp_rotbak.at<float>(idrow, idcol);
            if (dispval < min_temp) {
                min_temp = dispval;
            }
            if (dispval > max_temp) {
                max_temp = dispval;
            }
            float x = 0.0, y = 0.0 , z = 100;
            if (std::fabs(dispval) < 1e-30) {
                z = 100.0;
            } else {
                z = focal_length * base_line / dispval;
            }
            x = (idcol - cx) * z / focal_length;
            y = (idrow - cy) * z / focal_length;

            point_3d_img.at<Vec4f>(idrow, idcol)[0] = x;
            point_3d_img.at<Vec4f>(idrow, idcol)[1] = y;
            point_3d_img.at<Vec4f>(idrow, idcol)[2] = z;
            point_3d_img.at<Vec4f>(idrow, idcol)[3] = left_img_rotbak.at<uchar>(idrow, idcol);
        }
    }

    std::cout << "min temp " << min_temp << " max temp " << max_temp << "\n";

    SaveToTxtFile(point_3d_img, file_out_pointcloud);
//    cv::waitKey(0);

    return 0;
}