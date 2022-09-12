#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import  Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PointStamped
from time import sleep
import tf

from rospy.numpy_msg import numpy_msg
import ros_numpy


import cv2

#(msg_types={'geometry_msgs/PoseStamped': 'd3812c3cbc69362b77dc0b19b345f8f5', 
# 'sensor_msgs/Image': '060021388200f6f0f447d0fcd9c64743', 
# 'sensor_msgs/Imu': '6a62c6daae103f4ff57a132d6f95cec2'}, 
# topics=
# {'/zed2/zed_node/depth/depth_registered': TopicTuple(msg_type='sensor_msgs/Image', message_count=487, connections=1, frequency=10.498134676249629), 
# '/zed2/zed_node/imu/data': TopicTuple(msg_type='sensor_msgs/Imu', message_count=6485, connections=1, frequency=425.3426630159213), 
# '/zed2/zed_node/left/image_rect_color': TopicTuple(msg_type='sensor_msgs/Image', message_count=418, connections=1, frequency=10.00494724764624), 
# '/zed2/zed_node/pose': TopicTuple(msg_type='geometry_msgs/PoseStamped', message_count=628, connections=1, frequency=14.898936831523496), 
# '/zed2/zed_node/rgb/image_rect_color': TopicTuple(msg_type='sensor_msgs/Image', message_count=409, connections=1, frequency=10.073562417815992), 
# '/zed2/zed_node/right/image_rect_color': TopicTuple(msg_type='sensor_msgs/Image', message_count=366, connections=1, frequency=10.072751986666699)})

class Camera_output:

    def __init__(self, node = False):
        self.img_count = 7
        self.img_rgb = None
        self.img_depth = None
        self.Px = 554.25
        self.Py = 554.25
        self.count_bgr = 0
        self.count_bgr_r = 0
        self.count_bgr_l = 0
        self.count_depth = 0


        #NODE
        if node:
            rospy.init_node('Depth_camera_processing', anonymous= True)

        #SUBSCRIBER
        #RGBA
        self.get_rgb_image = rospy.Subscriber('/depth_camera/rgb/image_raw', Image, self.get_rgb)
        #DEPTH
        self.get_depth_image = rospy.Subscriber('/depth_camera/depth/image_raw', numpy_msg(Image), self.get_depth)
        #intrinsic parameters
        self.depth_camera_info = rospy.Subscriber('/depth_cameracamera/color/camera_info', CameraInfo, self.get_info)

        #point cloud

        rospy.Subscriber("/depth_camera/depth/points", PointCloud2, self.callback)

        sleep(2)
        self.listener = tf.TransformListener()

    def get_depth(self, depth_data):
        # self.img_depth = np.frombuffer(depth_data.data, dtype=np.float32).reshape(depth_data.height, depth_data.width, -1)
        img_depth = np.frombuffer(depth_data.data, dtype=np.float32).reshape(depth_data.height, depth_data.width, -1)
        img = np.array(img_depth).copy()
        img = np.nan_to_num(img, nan = -100)
        img = np.reshape(img, (img.shape[0], img.shape[1]))
        self.img_depth = img.copy()

    def get_rgb(self, rgb_data):
        self.img_rgb = np.frombuffer(rgb_data.data, dtype=np.uint8).reshape(rgb_data.height, rgb_data.width, -1)[:,:,0:3]
        self.img_bgr = self.img_rgb[...,::-1]


    def get_info(self, data):
        self.Px = data.P[0]
        self.Py = data.P[5]


    def callback(self, value):
        self.pc_arr = ros_numpy.numpify(value)


    def get_xyz(self, w, h):
        x = self.pc_arr['x'][w, h]
        y = self.pc_arr['y'][w, h]
        z = self.pc_arr['z'][w, h]
        return np.array([x, y, z])

    def get_global(self, x, y, z):
        ps = PointStamped()
        ps.header.frame_id = "tilt_link"
        ps.header.stamp = rospy.Time(0)
        ps.point.x = x
        ps.point.y = y
        ps.point.z = z
        mat = self.listener.transformPoint("/map", ps)

        return [mat.point.x, mat.point.y, mat.point.z]
        




        


if __name__ == '__main__':
    try:
            camera = Camera_output()
    except rospy.ROSInterruptException:
            pass
    sleep(2)
    camera.get_rgb_image
    camera.get_depth_image
    camera.depth_camera_info
    sleep(2)
    print(camera.Px)
    cv2.imshow('hmm', camera.img_bgr)
    cv2.imshow('hmmm', camera.img_depth)
    cv2.waitKey()
    cv2.destroyAllWindows()
    #np.save('depth.npy', np.array(camera.img_depth))
    