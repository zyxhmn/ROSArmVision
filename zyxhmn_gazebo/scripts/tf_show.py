#!/usr/bin/env python3

import rospy
import tf2_ros

rospy.init_node('tf_listener_node')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

rate = rospy.Rate(1.0)
while not rospy.is_shutdown():
    frames = tfBuffer.all_frames_as_yaml()
    print(frames)
    rate.sleep()
