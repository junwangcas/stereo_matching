import sys
sys.path.append('../ransac_plane')
from fit_plane import fit_plane, filter_planes
from error_funcs import ransac_error, msac_error, mlesac_error
from plot_results import *
import open3d as o3d
import numpy as np

def SingleBestPlane(pcd) :
    error_functions = [ransac_error, msac_error, mlesac_error]
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

    if not pcd.has_points():
        raise FileNotFoundError("Couldn't load pointcloud")

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

    return pcd_sampled, best_plane, best_inliers

def GetPlanePc(pcd_sampled, best_inliers):
    pcd_plane = o3d.geometry.PointCloud()
    plane_pts_array = np.empty([0, 3])
    pts_size = len(best_inliers)
    for i in range(pts_size):
        is_inlier = best_inliers[i]
        if is_inlier:
            pt = pcd_sampled.points[i]
            pcd_plane.points.append(pt)
    return pcd_plane

def GetBestPlane(pc_o3d):
    pcd_sampled, best_plane, best_inliers = SingleBestPlane(pc_o3d)
    pcd_plane = GetPlanePc(pcd_sampled, best_inliers)
    return pcd_sampled, pcd_plane
