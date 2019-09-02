#!/usr/bin/env python

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import PoseStamped

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
    rospy.loginfo("from {} to {} is at x:{}, y:{}, z:{}, roll:{}, pitch:{}, yaw:{}".format(source, target, x, y, z, roll, pitch, yaw))

    if broadcaster:
        print "BEFORE CONVERSION rot: {}, {}, {}, {}".format(rot.x, rot.y, rot.z, rot.w)
        new_tf = TransformStamped()
        new_tf.header.frame_id = "base_footprint"
        new_tf.header.stamp = rospy.Time.now()
        new_tf.child_frame_id = "ballseen"
        new_tf.transform.translation.x = x
        new_tf.transform.translation.y = y
        print "roll, pitch, yaw is {}, {}, {}".format(roll, pitch, yaw)
        q = quaternion_from_euler(roll, pitch, yaw)
        new_tf.transform.rotation.x = q[0]
        new_tf.transform.rotation.y = q[1]
        new_tf.transform.rotation.z = q[2]
        new_tf.transform.rotation.w = q[3]
        print "AFTER CONVERSION q: {}, {}, {}, {}".format(q[0], q[1], q[2], q[3])
        broadcaster.sendTransform(new_tf)
        
        ps = PoseStamped()
        ps.header.frame_id = "base_footprint"
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]
        try:
            tfed_ps = buffer.transform(ps, "map", timeout=rospy.Duration(0.5))
        except Exception as e:
            rospy.logwarn(e)
            return
        tfed_frame_id = tfed_ps.header.frame_id
        tfed_x = tfed_ps.pose.position.x
        tfed_y = tfed_ps.pose.position.y
        tfed_z = tfed_ps.pose.position.z
        tfed_q = tfed_ps.pose.orientation
        tfed_a = euler_from_quaternion([tfed_q.x, tfed_q.y, tfed_q.z, tfed_q.w])
        rospy.loginfo("tfed_ps's frame_id is {}, position is x:{}, y{}, z{},roll: {}, pitch: {}, yaw: {}".format(tfed_frame_id, tfed_x, tfed_y, tfed_z, tfed_a[0], tfed_a[1], tfed_a[2]))


def main():
    rospy.init_node("test_tf_coordinate")
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(2))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        show_pos("map", "base_footprint", tf_buffer)
        show_pos("map", "ball",  tf_buffer)
        show_pos("base_footprint", "ball", tf_buffer, tf_broadcaster)
        show_pos("map", "ballseen", tf_buffer)
        rate.sleep()

if __name__ == "__main__":
    main()

