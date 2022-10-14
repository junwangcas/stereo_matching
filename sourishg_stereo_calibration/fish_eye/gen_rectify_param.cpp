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

void stereoRectify3416( InputArray K1, InputArray D1, InputArray K2, InputArray D2, const Size& imageSize,
                        InputArray _R, InputArray _tvec, OutputArray R1, OutputArray R2, OutputArray P1, OutputArray P2,
                        OutputArray Q, int flags, const Size& newImageSize, double balance, double fov_scale)
{
//    CV_INSTRUMENT_REGION();

    CV_Assert((_R.size() == Size(3, 3) || _R.total() * _R.channels() == 3) && (_R.depth() == CV_32F || _R.depth() == CV_64F));
    CV_Assert(_tvec.total() * _tvec.channels() == 3 && (_tvec.depth() == CV_32F || _tvec.depth() == CV_64F));


    cv::Mat aaa = _tvec.getMat().reshape(3, 1);

    Vec3d rvec; // Rodrigues vector
    if (_R.size() == Size(3, 3))
    {
        cv::Matx33d rmat;
        _R.getMat().convertTo(rmat, CV_64F);
        rvec = Affine3d(rmat).rvec();
    }
    else if (_R.total() * _R.channels() == 3)
        _R.getMat().convertTo(rvec, CV_64F);

    Vec3d tvec;
    _tvec.getMat().convertTo(tvec, CV_64F);

    // rectification algorithm
    rvec *= -0.5;              // get average rotation

    Matx33d r_r;
    Rodrigues(rvec, r_r);  // rotate cameras to same orientation by averaging

    Vec3d t = r_r * tvec; /// eppVec
    Vec3d uu;
    bool is_vertical;
    if (std::abs(t[0]) > std::abs(t[1])) {
        uu << (t[0] > 0 ? 1 : -1), 0, 0;
        is_vertical = false;
    } else {
        uu << 0, (t[1] > 0 ? 1 : -1), 0;
        is_vertical = true;
    }

    // printf("is_vertical: %d\n", is_vertical);
    // printf("t: [%f %f %f]\n", t[0], t[1], t[2]);

    // calculate global Z rotation
    Vec3d ww = t.cross(uu);
    double nw = norm(ww);
    if (nw > 0.0) {
        if (!is_vertical) {
            ww *= acos(fabs(t[0]) / cv::norm(t)) / nw;
        } else {
            ww *= acos(fabs(t[1]) / cv::norm(t)) / nw;
        }
    }

    // printf("ww: [%f %f %f]\n", ww[0], ww[1], ww[2]);

    Matx33d wr;
    Rodrigues(ww, wr);

    // apply to both views
    Matx33d ri1 = wr * r_r.t();
    Mat(ri1, false).convertTo(R1, R1.empty() ? CV_64F : R1.type());
    Matx33d ri2 = wr * r_r;
    Mat(ri2, false).convertTo(R2, R2.empty() ? CV_64F : R2.type());
    Vec3d tnew = ri2 * tvec;

    // printf("tnew: [%f %f %f]\n", tnew[0], tnew[1], tnew[2]);

    // calculate projection/camera matrices. these contain the relevant rectified image internal params (fx, fy=fx, cx, cy)
    Matx33d newK1, newK2;
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K1, D1, imageSize, R1, newK1, balance, newImageSize, fov_scale);
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K2, D2, imageSize, R2, newK2, balance, newImageSize, fov_scale);

    double fc_new = std::min(newK1(1,1), newK2(1,1));
    Point2d cc_new[2] = { Vec2d(newK1(0, 2), newK1(1, 2)), Vec2d(newK2(0, 2), newK2(1, 2)) };

    // Vertical focal length must be the same for both images to keep the epipolar constraint use fy for fx also.
    // For simplicity, set the principal points for both cameras to be the average
    // of the two principal points (either one of or both x- and y- coordinates)
    if( flags & cv::CALIB_ZERO_DISPARITY )
        cc_new[0] = cc_new[1] = (cc_new[0] + cc_new[1]) * 0.5;
    else
        cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;

    Mat(Matx34d(fc_new, 0, cc_new[0].x, 0,
                0, fc_new, cc_new[0].y, 0,
                0,      0,           1, 0), false).convertTo(P1, P1.empty() ? CV_64F : P1.type());

    if (!is_vertical) {
        Mat(Matx34d(fc_new, 0, cc_new[1].x, tnew[0] * fc_new, // baseline * focal length;,
                    0, fc_new, cc_new[1].y, 0,
                    0, 0, 1, 0), false).convertTo(P2, P2.empty() ? CV_64F : P2.type());
    } else {
        Mat(Matx34d(fc_new, 0, cc_new[1].x, tnew[1] * fc_new, // baseline * focal length;,
                    0, fc_new, cc_new[1].y, 0,
                    0, 0, 1, 0), false).convertTo(P2, P2.empty() ? CV_64F : P2.type());
    }
    if (Q.needed())
        if(!is_vertical) {
            Mat(Matx44d(1, 0, 0, -cc_new[0].x,
                        0, 1, 0, -cc_new[0].y,
                        0, 0, 0, fc_new,
                        0, 0, -1. / tnew[0], (cc_new[0].x - cc_new[1].x) / tnew[0]), false).convertTo(Q,
                                                                                                      Q.empty() ? CV_64F
                                                                                                                : Q.depth());
        } else {
            Mat(Matx44d(1, 0, 0, -cc_new[0].x,
                        0, 1, 0, -cc_new[0].y,
                        0, 0, 0, fc_new,
                        0, 0, -1. / tnew[1], (cc_new[0].y - cc_new[1].y) / tnew[1]), false).convertTo(Q,
                                                                                                      Q.empty() ? CV_64F
                                                                                                                : Q.depth());
        }
}

int main(int argc, char const *argv[])
{
    cv::Size img_size(640, 480);

    // 输入标定文件，生成rectify所需要R1, P1, R2, P2并保存
    char* calib_file;
    char* rectify_file;

    static struct poptOption options[] = {
            { "calib_file",'c',POPT_ARG_STRING,&calib_file,0,"Stereo calibration file","STR" },
            { "rectify_filename",'o',POPT_ARG_STRING,&rectify_file,0,"rectify file","STR" },
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

    cv::FileStorage fs1(calib_file, cv::FileStorage::READ);
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

    stereoRectify3416(K1, D1, K2, D2, img_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, img_size, 0.5, 0.5);
    PrintMat(R1, "R1");
    PrintMat(R2, "R2");
    PrintMat(P1, "P1");
    PrintMat(P2, "P2");
    PrintMat(Q, "Q");

    cv::FileStorage fs_q(rectify_file, cv::FileStorage::WRITE);
    fs_q << "K1" << K1 << "K2" << K2 << "D1" << D1 << "D2" << D2;
    fs_q << "R1" << R1 << "P1" << P1 << "R2" << R2 << "P2" << P2 << "Q" << Q;
    fs_q.release();
    return 0;
}