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

void stereoRectifyVertical( InputArray K1, InputArray D1, InputArray K2, InputArray D2, const Size& imageSize,
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

     printf("is_vertical: %d\n", is_vertical);
     printf("t: [%f %f %f]\n", t[0], t[1], t[2]);

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

    printf("ww: [%f %f %f]\n", ww[0], ww[1], ww[2]);

    Matx33d wr;
    Rodrigues(ww, wr);

    // apply to both views
    Matx33d ri1 = wr * r_r.t();
    Mat(ri1, false).convertTo(R1, R1.empty() ? CV_64F : R1.type());
    Matx33d ri2 = wr * r_r;
    Mat(ri2, false).convertTo(R2, R2.empty() ? CV_64F : R2.type());
    Vec3d tnew = ri2 * tvec;

     printf("tnew: [%f %f %f]\n", tnew[0], tnew[1], tnew[2]);

    // calculate projection/camera matrices. these contain the relevant rectified image internal params (fx, fy=fx, cx, cy)
    Matx33d newK1, newK2;
//    PrintMat(K1, "K1");
//    PrintMat(D2, "D1");
//    PrintMat(R1, "R1");
//    std::cout << "blance " << balance << " fov_scale " <<fov_scale << " image size " << imageSize << " new Image Size " << newImageSize << "\n";
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K1, D1, imageSize, R1, newK1, balance, newImageSize, fov_scale);

//    PrintMat(newK1, "new K1");
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K2, D2, imageSize, R2, newK2, balance, newImageSize, fov_scale);
//    exit(1);


    std::cout << "newk 1 " << newK1 << " \n new k2 " << newK2 << "\n";
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

void ShowHconImgs(cv::Mat &img1, cv::Mat &img2, std::string window_name)
{
    cv::Mat left_img_rotate, right_img_rotate, left_right_rect_show;
    cv::rotate(img1, left_img_rotate, cv::ROTATE_90_CLOCKWISE);
    cv::rotate(img2, right_img_rotate, cv::ROTATE_90_CLOCKWISE);
    cv::hconcat(left_img_rotate, right_img_rotate, left_right_rect_show);
    cv::imshow(window_name, left_right_rect_show);
}

void GetDispartyMap(cv::Mat &left_img_rotate, cv::Mat &right_img_rotate, cv::Mat &disparity)
{
    enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };
    int numberOfDisparities = ((left_img_rotate.cols / 4) + 15) & -16;
    printf("********* number of disparities: %d \n",numberOfDisparities);
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create();
    sgbm->setPreFilterCap(31);
    int SADWindowSize = 5; // 9;
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);
    int cn = left_img_rotate.channels();
    int min_disp = 1; //0.2; // 3; //1; //5;
    sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(min_disp);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(5);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);

    int alg = STEREO_SGBM;
    if (alg == STEREO_HH)
        sgbm->setMode(cv::StereoSGBM::MODE_HH);
    else if (alg == STEREO_SGBM)
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
    else if (alg == STEREO_3WAY)
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
    sgbm->compute(left_img_rotate, right_img_rotate, disparity);


    cv::Mat disparity_color, disp;
    disparity.convertTo(disp, CV_8U, 255.0 / numberOfDisparities/16.0);
    //cv::applyColorMap(disp, disparity_color, cv::COLORMAP_JET);
    cv::applyColorMap(disp, disparity_color, cv::COLORMAP_RAINBOW);

    cv::imshow("RAW RECTIFIED DISP", disparity_color);
    cv::waitKey(200000000);
}


int main(int argc, char const *argv[])
{
  char* leftimg_filename;
  char* rightimg_filename;
  char* calib_file;
  char* leftout_filename;
  char* rightout_filename;

  static struct poptOption options[] = {
    { "leftimg_filename",'l',POPT_ARG_STRING,&leftimg_filename,0,"Left imgage path","STR" },
    { "rightimg_filename",'r',POPT_ARG_STRING,&rightimg_filename,0,"Right image path","STR" },
    { "calib_file",'c',POPT_ARG_STRING,&calib_file,0,"Stereo calibration file","STR" },
    { "leftout_filename",'L',POPT_ARG_STRING,&leftout_filename,0,"Left undistorted imgage path","STR" },
    { "rightout_filename",'R',POPT_ARG_STRING,&rightout_filename,0,"Right undistorted image path","STR" },
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

  std::cout << img1.size() << "\n";
    {
        stereoRectify3416(K1, D1, K2, D2, img1.size(), R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, img1.size(), 0.5, 0.5);
//        cv::fisheye::stereoRectify(K1, D1, K2, D2, img1.size(), R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, img1.size(), 0.5, 1.2);
//        stereoRectifyVertical(K1, D1, K2, D2, img1.size(), R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, img1.size(), 0.5, 1.2);
        PrintMat(R1, "R1");
        PrintMat(R2, "R2");
        PrintMat(P1, "P1");
        PrintMat(P2, "P2");
        PrintMat(Q, "Q");
    }

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
        imwrite(leftout_filename, imgU1);
        imwrite(rightout_filename, imgU2);
        imwrite("left_rect_rot.pgm", left_img_rotate);
        imwrite("right_rect_rot.pgm", right_img_rotate);

        cv::FileStorage fs_q("rectify_q.yml", cv::FileStorage::WRITE);
        fs_q << "Q" << Q;
        fs_q.release();
    }

    {
        cv::Mat disparity;
        GetDispartyMap(left_img_rotate, right_img_rotate, disparity);
    }


  cv::waitKey(0);
  return 0;
}