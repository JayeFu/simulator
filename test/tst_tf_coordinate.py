#!/usr/bin/env python

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped

def show_pos(target, source, buffer, broadcaster=None):
    try:
        tf_stamped = buffer.lookup_transform(target, source, rospy.Time(0))
    except Exception as e:
        rospy.logwarn(e)
        return
    trans = tf_stamped.transform.translation
    rot = tf_stamped.transform.rotation
    x = trans.x
    y = trans.y
    z = trans.z

    a = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
    roll = a[0]
    pitch = a[1]
    yaw = a[2]
    rospy.loginfo("{} relative to {} is at x:{}, y:{}, z:{}, roll:{}, pitch:{}, yaw:{}".format(target, source, x, y, z, roll, pitch, yaw))

    if broadcaster:
        new_tf = TransformStamped()
        new_tf.header.frame_id = "base_footprint"
        new_tf.header.stamp = rospy.Time.now()
        new_tf.child_frame_id = "ballseen"
        new_tf.transform.translation.x = x
        new_tf.transform.translation.y = y
        q = quaternion_from_euler(roll, yaw, pitch)
        new_tf.transform.rotation.x = q[0]
        new_tf.transform.rotation.y = q[1]
        new_tf.transform.rotation.z = q[2]
        new_tf.transform.rotation.w = q[3]
        broadcaster.sendTransform(new_tf)


def main():
    rospy.init_node("test_tf_coordinate")
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(2))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        show_pos("base_footprint", "map", tf_buffer)
        show_pos("ball", "map", tf_buffer)
        show_pos("ball", "base_footprint", tf_buffer, tf_broadcaster)
        show_pos("ballseen", "map", tf_buffer)
        rate.sleep()

if __name__ == "__main__":
    main()

