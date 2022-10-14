import open3d as o3d
import argparse
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from matplotlib import style
import numpy as np

def parse_args():
    """
    :return:进行参数的解析
    """
    description = "you should add those parameter"                   # 步骤二
    parser = argparse.ArgumentParser(description=description)        # 这些参数都有默认值，当调用parser.print_help()或者运行程序时由于参数不正确(此时python解释器其实也是调用了pring_help()方法)时，
                                                                     # 会打印这些描述信息，一般只需要传递description参数，如上。
    help = "The path of address"
    parser.add_argument('--txt_file',help = help)                   # 步骤三，后面的help是我的描述
    args = parser.parse_args()                                       # 步骤四
    return args

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

def WriteO3d(xyzs_np, pointcloud_file):
    pc_o3d = o3d.geometry.PointCloud()
    pc_o3d.points = o3d.utility.Vector3dVector(xyzs_np)
    o3d.visualization.draw_geometries([pc_o3d])

def Visualize3d(xyzs_np):
    style.use('ggplot')
    fig = plt.figure()
    ax1 = fig.add_subplot(111, projection='3d')
    x = xyzs_np[:, 0]
    y = xyzs_np[:, 1]
    z = xyzs_np[:, 2]
    ax1.scatter(x, y, z, c='m', marker='o')
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_zlabel('z')
    ax1.set_zlim(-1, 6)
    plt.show()

if __name__ == '__main__':
    args = parse_args()
    print(args.txt_file)
    txt_file = args.txt_file

    xyzs = GetXYZs(txt_file)
    xyzs_np = ToNpArray(xyzs)
    Visualize3d(xyzs_np)
    # WriteO3d(xyzs_np, "point_cloud.pc")




