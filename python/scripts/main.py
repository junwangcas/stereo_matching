import os
import sys
sys.path.append('../ransac_plane')
sys.path.append('../ransac_plane/expriment')

from update_binary import update_binary

# dataset_path = '/sdcard/stereo_matching/rongjie_hmd/'
# dataset_enum = ['controller_2022-07-06-08-54-25', ]

target_path = './working_space/data/'
left_img_file =  target_path + 'cases/case2/' + 'left.bmp'
right_img_file = target_path + 'cases/case2/' + 'right.bmp'


# def GetLRFile():



update_binary()

do_rectify_param = True
do_rectify_img = True
do_elas = True
do_gen_pointcloud = True
do_gen_plane = True
do_visualization = True

if do_rectify_param:
    cmd_str = './working_space/gen_rectify_param -c ' + target_path + 'cam_intrinsics.yml' + ' -o ' +  target_path + 'rectify_q.yml'
    print(cmd_str)
    os.system(cmd_str)


if do_rectify_img:
    cmd_str = './working_space/gen_rectify_img -l ' + left_img_file + ' -r ' + right_img_file + ' -c ' +  target_path + 'rectify_q.yml'
    cmd_str = cmd_str + ' -L ' + target_path + 'left.pgm' + ' -R ' + target_path + 'right.pgm' + ' -H ' + target_path + 'left_right.pgm'
    print(cmd_str)
    os.system(cmd_str)


if do_elas:
    cmd_str = './working_space/elas ' + target_path + 'left.pgm ' + target_path + 'right.pgm'
    print(cmd_str)
    os.system(cmd_str)


if do_gen_pointcloud:
    cmd_str = './working_space/gen_pointcloud_my_imp -d ' + target_path + 'left_disp_float.tiff' + ' -c ' + target_path + 'rectify_q.yml ' + ' -L ' + target_path + 'point_cloud.txt'
    cmd_str = cmd_str + ' -i ' + target_path + 'left.pgm'
    print(cmd_str)
    os.system(cmd_str)


if do_gen_plane:
    pythonpath = '/usr/bin/python3.8'
    cmd_str = pythonpath + ' ../ransac_plane/expriment/parse_plane.py ' + '--txt_file ' + target_path + 'point_cloud.txt'
    cmd_str = cmd_str + ' --downsample_file ' + target_path + 'down_sample_pts.txt'
    cmd_str = cmd_str + ' --plane_file ' + target_path + 'plane_pts.txt'
    os.system(cmd_str)



if do_visualization:
    pythonpath = '/usr/bin/python3.8'
    cmd_str = pythonpath + ' ./ros/visualize_point_cloud.py' + ' --txt_file ' + target_path + 'point_cloud.txt'
    os.system(cmd_str)


###############################back up####################
# ./gen_pointcloud -d left_disp_float.bmp -c rectify_q.yml -L point_cloud.txt
# cmd_str = './working_space/gen_pointcloud -d ' + target_path + 'left_disp_float.bmp' + ' -c ' + target_path + 'rectify_q.yml ' + ' -L ' + target_path + 'point_cloud.txt'
# print(cmd_str)
# os.system(cmd_str)

# do_visualization = True
# if do_visualization:
#     pythonpath = '~/miniconda3/envs/ransac/bin/python'
#     cmd_str = pythonpath + ' ../ransac_plane/expriment/trans_txt_o3d.py ' + ' --txt_file ' + target_path + 'point_cloud.txt'
#     os.system(cmd_str)
