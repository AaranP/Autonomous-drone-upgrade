#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
import tf


class odom2post:

    def __init__(self):
        self.pose_pub = rospy.Publisher('/cam_pose', PoseStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber('/vins_fusion/camera_pose', Odometry, self.odom_cb, queue_size=1)
        self.pose = PoseStamped()

    def odom_cb(self, msg):
        self.pose.header = msg.header
        self.pose.pose = msg.pose.pose
        self.pose.header = msg.header
        self.pose_pub.publish(self.pose)

if __name__ == '__main__':
    rospy.init_node('odom_to_post')
    odom_to_post = odom2post()
    rospy.spin()
