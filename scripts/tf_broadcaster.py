#!/usr/bin/env python
import numpy as np
import rospy
import tf2_ros
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class tf_broadcaster():
    def __init__(self):

        # ROS infraestructure
        self.pose_sub = rospy.Subscriber("/mavros/global_position/local", Odometry, self.callback)
        self.laser_sub = rospy.Subscriber("/laser/scan", LaserScan, self.callback_laser)

    def callback(self, msg):
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.frame_id = "world"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "/if750a/base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        br.sendTransform(t)

    def callback_laser(self, msg):
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.frame_id = "/if750a/rplidar_link"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "rplidar_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        br.sendTransform(t)




if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    try:
        node = tf_broadcaster()
        rate = rospy.Rate(5)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
