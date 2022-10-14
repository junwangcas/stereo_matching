cmake_minimum_required(VERSION 2.8.11)
project(CALIB_CAM)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(OpenCV 3.4.16 REQUIRED)
include_directories($(OpenCV_INCLUDE_DIRS))

add_executable(calibrate calib_intrinsic.cpp popt_pp.h)
target_link_libraries(calibrate ${OpenCV_LIBS} "-lpopt")

add_executable(read read_images.cpp)
target_link_libraries(read ${OpenCV_LIBS} "-lpopt")

add_executable(calibrate_stereo calib_stereo.cpp)
target_link_libraries(calibrate_stereo ${OpenCV_LIBS} "-lpopt")

add_executable(undistort_rectify undistort_rectify.cpp)
target_link_libraries(undistort_rectify ${OpenCV_LIBS} "-lpopt")

add_executable(undistort_rectify_fisheye fish_eye/undistort_rectify_fisheye.cpp)
target_link_libraries(undistort_rectify_fisheye ${OpenCV_LIBS} "-lpopt")

add_executable(gen_rectify_param fish_eye/gen_rectify_param.cpp)
target_link_libraries(gen_rectify_param ${OpenCV_LIBS} "-lpopt")

add_executable(gen_rectify_img fish_eye/gen_rectify_img.cpp)
target_link_libraries(gen_rectify_img ${OpenCV_LIBS} "-lpopt")

add_executable(gen_pointcloud fish_eye/gen_pointcloud.cpp)
target_link_libraries(gen_pointcloud ${OpenCV_LIBS} "-lpopt")

add_executable(gen_pointcloud_my_imp fish_eye/gen_pointcloud_my_imp.cpp)
target_link_libraries(gen_pointcloud_my_imp ${OpenCV_LIBS} "-lpopt")