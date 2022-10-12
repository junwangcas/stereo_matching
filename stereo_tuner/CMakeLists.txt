cmake_minimum_required(VERSION 2.8.11)
project(stereo_tuner)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(OpenCV 3.4.16 REQUIRED)
include_directories($(OpenCV_INCLUDE_DIRS))

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
include_directories(${GTK3_INCLUDE_DIRS})
link_directories(${GTK3_LIBRARY_DIRS})


add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBS} ${GTK3_LIBRARIES})

install(TARGETS main
        DESTINATION ../main)
#add_executable(calibrate calib_intrinsic.cpp popt_pp.h)
#target_link_libraries(calibrate ${OpenCV_LIBS} "-l")
#
#add_executable(read read_images.cpp)
#target_link_libraries(read ${OpenCV_LIBS} "-lpopt")
#
#add_executable(calibrate_stereo calib_stereo.cpp)
#target_link_libraries(calibrate_stereo ${OpenCV_LIBS} "-lpopt")
#
#add_executable(undistort_rectify undistort_rectify.cpp)
#target_link_libraries(undistort_rectify ${OpenCV_LIBS} "-lpopt")
#
#add_executable(undistort_rectify_fisheye fish_eye/undistort_rectify_fisheye.cpp)
#target_link_libraries(undistort_rectify_fisheye ${OpenCV_LIBS} "-lpopt")