#!/usr/bin/python3.8
import message_filters
from sensor_msgs.msg import Image, Imu
import rospy

rospy.init_node('synchronize', anonymous=True)

image1_pub = rospy.Publisher("image1", Image, queue_size=1)
image2_pub = rospy.Publisher("image2", Image, queue_size=1)
imu_pub = rospy.Publisher("imu", Imu, queue_size=1)



def callback(image1, image2, imu):
# Solve all of perception here...
    image1_pub.publish(image1)
    image2_pub.publish(image2)
    imu_pub.publish(imu)
    # print(image1)

image1_sub = message_filters.Subscriber('/camera/infra1/image_rect_raw', Image)
image2_sub = message_filters.Subscriber('/camera/infra2/image_rect_raw', Image)
imu_sub = message_filters.Subscriber('mavros/imu/data', Imu)

ts = message_filters.ApproximateTimeSynchronizer([image1_sub, image2_sub, imu_sub], 10, 0.01)
ts.registerCallback(callback)
rospy.spin()