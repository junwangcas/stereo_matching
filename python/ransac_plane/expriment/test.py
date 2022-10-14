

import os

data_path = '/home/junwangcas/Documents/stereo_matching/stereo_matching_code/python/scripts/working_space/data/'

pythonpath = '/usr/bin/python3.8'
cmd_str = pythonpath + ' parse_plane.py' + ' --txt_file ' + data_path + 'point_cloud.txt'
print(cmd_str)
os.system(cmd_str)

--txt_file /home/junwangcas/Documents/stereo_matching/stereo_matching_code/python/scripts/working_space/data/point_cloud.txt --downsample_file /home/junwangcas/Documents/stereo_matching/stereo_matching_code/python/scripts/working_space/data/down_sample.txt --plane_file /home/junwangcas/Documents/stereo_matching/stereo_matching_code/python/scripts/working_space/data/plane_pts.txt