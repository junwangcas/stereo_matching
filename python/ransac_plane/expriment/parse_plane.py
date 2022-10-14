import argparse
from read_txt_pc import ReadTxtPc
from o3d_pc import SingleBestPlane, GetBestPlane

def parse_args():
    """
    :return:进行参数的解析
    """
    description = "you should add those parameter"                   # 步骤二
    parser = argparse.ArgumentParser(description=description)        # 这些参数都有默认值，当调用parser.print_help()或者运行程序时由于参数不正确(此时python解释器其实也是调用了pring_help()方法)时，
                                                                     # 会打印这些描述信息，一般只需要传递description参数，如上。
    help = "The path of address"
    parser.add_argument('--txt_file',help = help)                   # 步骤三，后面的help是我的描述
    parser.add_argument('--downsample_file', help = help)
    parser.add_argument('--plane_file', help=help)
    args = parser.parse_args()                                       # 步骤四
    return args

def WriteTxt(pc_o3d, txt_file):
    file_handle = open(txt_file, 'w')
    for i in range(len(pc_o3d.points)):
        pt = pc_o3d.points[i]
        line_str = str(pt[0]) + ', ' + str(pt[1]) + ', ' + str(pt[2]) + '\n'
        file_handle.write(line_str)
    file_handle.close()

if __name__ == '__main__':
    args = parse_args()
    print(args.txt_file)
    txt_file = args.txt_file
    downsample_file = args.downsample_file
    plane_file = args.plane_file

    pc_o3d = ReadTxtPc(txt_file)
    pc_downsample, pc_plane = GetBestPlane(pc_o3d)
    print('down sample pts ', pc_downsample, ' plane points ', pc_plane)
    WriteTxt(pc_downsample, downsample_file)
    WriteTxt(pc_plane, plane_file)

