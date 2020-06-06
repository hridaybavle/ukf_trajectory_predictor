#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from tf2_msgs.msg import TFMessage
import message_filters


def cb_1(msg):
    global first_stamp, now, FILTER_TF
    for i, tf in enumerate(msg.transforms):
        try:
            first_stamp[tf.header.frame_id]
        except:
            first_stamp[tf.header.frame_id] = tf.header.stamp

        #tf.header.stamp -= first_stamp[tf.header.frame_id]
        #print(tf.header.frame_id)
        #local = tf.header.stamp - first_stamp[tf.header.frame_id]
        #print(local)
        #tf.header.stamp += now
        if FILTER_TF and ("drone_" in tf.child_frame_id):
           tf.child_frame_id = "disabled_" + tf.child_frame_id
           #print(tf.header.frame_id)
        tf.header.stamp = rospy.Time.now()
    pub_1.publish(msg)


rospy.init_node('hoge')
first_stamp = {}

rospy.sleep(1)
now = rospy.Time.now()
FILTER_TF = False


pub_1 = rospy.Publisher('/tf', TFMessage, queue_size=1)

sub_1 = rospy.Subscriber('/tf_old', TFMessage, cb_1, queue_size=1)

rospy.spin()
