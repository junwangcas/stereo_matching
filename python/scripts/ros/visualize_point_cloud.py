import argparse
import numpy as np

import rospy
import sensor_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32

def TestPublisher():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)  # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        return

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
        if len(line_split) < 4:
            continue
        tuplexyz = (float(line_split[0]), float(line_split[1]), float(line_split[2]), float(line_split[3]))
        xyzs.append(tuplexyz)
    print("get xyzs: ", len(xyzs))
    return xyzs

# def ToNpArray(xyzs):
#     xyzs_np = np.random.rand(len(xyzs), 3)
#     for i in range(len(xyzs)):
#         xyzs_np[i, 0] = xyzs[i][0]
#         xyzs_np[i, 1] = xyzs[i][1]
#         xyzs_np[i, 2] = xyzs[i][2]
#     return xyzs_np

# def Visualize3d(xyzs_np):
#     style.use('ggplot')
#     fig = plt.figure()
#     ax1 = fig.add_subplot(111, projection='3d')
#     x = xyzs_np[:, 0]
#     y = xyzs_np[:, 1]
#     z = xyzs_np[:, 2]
#     ax1.scatter(x, y, z, c='m', marker='o')
#     ax1.set_xlabel('x')
#     ax1.set_ylabel('y')
#     ax1.set_zlabel('z')
#     ax1.set_zlim(-1, 6)
#     plt.show()

def GenPointcloud(xyzs):
    point_cloud_msg = PointCloud()
    point_cloud_msg.header.stamp = rospy.Time.now()
    point_cloud_msg.header.frame_id = 'map'
    channel_msg = ChannelFloat32()
    point_cloud_msg.channels.append(channel_msg)
    point_cloud_msg.channels[0].name = "intensities"
    # point_cloud_msg.channels[0].values.resize(len(xyzs));

    for i in range(len(xyzs)):
        xyz = xyzs[i]
        point = Point32()
        point.x = xyz[0]
        point.y = xyz[1]
        point.z = xyz[2]
        point_cloud_msg.points.append(point)
        point_cloud_msg.channels[0].values.append(xyz[3])

    return point_cloud_msg

def PubPointcloud(pc_msg):
    pub = rospy.Publisher('point_cloud_raw', sensor_msgs.msg.PointCloud, queue_size=10)
    rate = rospy.Rate(0.1)  # 10hz
    cout = 0
    while not rospy.is_shutdown():
        cout = cout + 1
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(pc_msg)
        rate.sleep()
        if cout > 5:
            break

if __name__ == '__main__':
    args = parse_args()
    print(args.txt_file)
    txt_file = args.txt_file

    xyzs = GetXYZs(txt_file)

    rospy.init_node('python_publisher', anonymous=True)
    pc_msg = GenPointcloud(xyzs)
    PubPointcloud(pc_msg)
    # trans
    # xyzs_np = ToNpArray(xyzs)
    # Visualize3d(xyzs_np)
    # WriteO3d(xyzs_np, "point_cloud.pc")


# --txt_file /home/junwangcas/Documents/stereo_matching/stereo_matching_code/python/scripts/working_space/data/point_cloud.txt