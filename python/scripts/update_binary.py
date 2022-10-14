import os


def update_binary():
    code_path = "/home/junwangcas/Documents/stereo_matching/stereo_matching_code/"
    target_path = 'working_space/'

    binary_stereo_calibration_path = code_path + "/sourishg_stereo_calibration/cmake-build-debug"
    cmd_str = 'cp ' + binary_stereo_calibration_path + "/gen_rectify_param " +  target_path
    os.system(cmd_str)
    cmd_str = 'cp ' + binary_stereo_calibration_path + "/gen_rectify_img " + target_path
    os.system(cmd_str)
    cmd_str = 'cp ' + binary_stereo_calibration_path + "/gen_pointcloud " + target_path
    os.system(cmd_str)
    cmd_str = 'cp ' + binary_stereo_calibration_path + "/gen_pointcloud_my_imp " + target_path
    os.system(cmd_str)

    cmd_str = 'cp ' + code_path + "/temp/cmake-build-debug/elas " + target_path
    os.system(cmd_str)


if __name__ == '__main__':
    update_binary()




