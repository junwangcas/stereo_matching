#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

from fit_plane import fit_plane, filter_planes
from error_funcs import ransac_error, msac_error, mlesac_error
from plot_results import *
# 一个main函数完成所有操作

def SingleBestPlane(current_path, error_functions) :
    # Selects which single-plane file to use；选择处理哪个点云
    pointcloud_idx = 2
    #########################################################
    # RANSAC parameters: ransac参数
    confidence = 0.85
    # m单位
    inlier_threshold = 0.02
    min_sample_distance = 0.8
    error_function_idx = 0

    # 下采样的大小
    voxel_size = 0.005
    #########################################################

    pcd = o3d.io.read_point_cloud(str(current_path.joinpath("pointclouds/image00")) + str(pointcloud_idx) + ".pcd")
    if not pcd.has_points():
        raise FileNotFoundError("Couldn't load pointcloud in " + str(current_path))

    # Down-sample the loaded point cloud to reduce computation time
    # pcd_sampled = pcd.uniform_down_sample(13) 下采样
    pcd_sampled = pcd.voxel_down_sample(voxel_size=voxel_size)

    # Apply plane-fitting algorithm 平面拟合， 最佳平面、最佳inliers, 迭代次数
    best_plane, best_inliers, num_iterations = fit_plane(pcd=pcd_sampled,
                                                         confidence=confidence,
                                                         inlier_threshold=inlier_threshold,
                                                         min_sample_distance=min_sample_distance,
                                                         error_func=error_functions[error_function_idx])

    # Plot the result； 坐标系红绿蓝xyz.
    plot_dominant_plane(pcd_sampled, best_inliers, best_plane)

def MultiPlanes(current_path, error_functions):
    #########################################################
    # 以下为提取多个平面
    # Multi-Plane parameters
    multi_plane_names = ['desk', 'door', 'kitchen']
    multi_plane_idx = 2

    # RANSAC parameters: 与单个平面的提取相比，多了一个min_points_prop参数，剩下的都是一样的
    min_points_prop = 0.04
    confidence_multi = 0.9999
    inlier_threshold_multi = 0.03
    min_sample_distance_multi = 0.1
    error_function_idx_multi = 0

    voxel_size_multi = 0.01

    #########################################################

    # Read Pointcloud for multiple plane detection

    pcd_multi = o3d.io.read_point_cloud(
        str(current_path.joinpath("pointclouds/" + multi_plane_names[multi_plane_idx] + ".pcd")))
    if not pcd_multi.has_points():
        raise FileNotFoundError("Couldn't load pointcloud in " + str(current_path))

    # Down-sample the loaded point cloud to reduce computation time
    pcd_multi_sampled = pcd_multi.uniform_down_sample(10)
    # pcd_multi_sampled = pcd_multi.voxel_down_sample(voxel_size=voxel_size_multi)

    plane_eqs, plane_pcds, filtered_pcd = filter_planes(pcd=pcd_multi_sampled,
                                                        min_points_prop=min_points_prop,
                                                        confidence=confidence_multi,
                                                        inlier_threshold=inlier_threshold_multi,
                                                        min_sample_distance=min_sample_distance_multi,
                                                        error_func=error_functions[error_function_idx_multi])

    plot_multiple_planes(plane_eqs=plane_eqs,
                         plane_pcds=plane_pcds,
                         filtered_pcd=filtered_pcd)

if __name__ == '__main__':
    # Read Pointcloud
    current_path = Path(__file__).parent
    error_functions = [ransac_error, msac_error, mlesac_error]
    # SingleBestPlane(current_path, error_functions)
    MultiPlanes(current_path, error_functions)


    