import open3d as o3d
import numpy as np

def GetXYZs(txt_file):
    xyzs = []
    file_handle = open(txt_file)
    for line in file_handle.readlines():
        line_split = line.split(',')
        if len(line_split) < 3:
            continue
        tuplexyz = (float(line_split[0]), float(line_split[1]), float(line_split[2]))
        xyzs.append(tuplexyz)
    print("get xyzs: ", len(xyzs))
    return xyzs

def ToNpArray(xyzs):
    xyzs_np = np.random.rand(len(xyzs), 3)
    for i in range(len(xyzs)):
        xyzs_np[i, 0] = xyzs[i][0]
        xyzs_np[i, 1] = xyzs[i][1]
        xyzs_np[i, 2] = xyzs[i][2]
    return xyzs_np

def ArrayToO3d(xyzs):
    xyzs_np = ToNpArray(xyzs)
    pc_o3d = o3d.geometry.PointCloud()
    pc_o3d.points = o3d.utility.Vector3dVector(xyzs_np)
    # o3d.visualization.draw_geometries([pc_o3d])
    return pc_o3d

def ReadTxtPc(txt_file):
    xyzs = GetXYZs(txt_file)
    pc_o3d = ArrayToO3d(xyzs)
    return pc_o3d


