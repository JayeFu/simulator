#!/usr/bin/env python

# Provide the infomation of the global position of the ball and the robot, which is definitely right
# The pos of the robot is Position2D, including header(std_msgs/Header-->seq(uint32), stamp(time), frame_id(string)), pose(geometry_msgs/Pose2D-->x, y, theta(all float64)), confidence(float32)
# The pos of the ball is BallRelative, including header(std_msgs/Header), ball_relative(geometry_msgs/Point-->x, y, z(all float64)), confidence(float32)

import rospy
import random
from math import pi as PI

from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
from geometry_msgs.msg import TransformStamped
from simulator.Pos2D import Pos2D

class environment:
    def __init__(self):
        self.robots_name = list()
        self.robots_pos = dict()
        self.ball_pos = Pos2D()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
    def add_robot(self, name):
        self.robots_name.append(name)
        new_x = 9.0 * random.random() - 4.5
        new_y = 6.0 * random.random() - 3.0
        new_theta = 2 * PI * random.random()
        self.robots_pos[name] = Pos2D(new_x, new_y, new_theta)

    def publish_robots(self):
        for name, pos in self.robots_pos.iteritems():
            new_tf = TransformStamped()
            new_tf.header.frame_id = "map"
            new_tf.header.stamp = rospy.Time.now()
            new_tf.child_frame_id = name
            new_tf.transform.translation.x = pos.x
            new_tf.transform.translation.y = pos.y
            new_theta = pos.theta
            q = quaternion_from_euler(0, 0, new_theta)
            new_tf.transform.rotation.x = q[0]
            new_tf.transform.rotation.y = q[1]
            new_tf.transform.rotation.z = q[2]
            new_tf.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(new_tf)

    def add_ball(self):
        self.ball_pos.x = 9.0 * random.random() - 4.5
        self.ball_pos.y = 6.0 * random.random() - 3.0
        self.ball_pos.theta = 0

    def publish_ball(self):
        new_tf = TransformStamped()
        new_tf.header.frame_id = "map"
        new_tf.header.stamp = rospy.Time.now()
        new_tf.child_frame_id = "ball"
        new_tf.transform.translation.x = self.ball_pos.x
        new_tf.transform.translation.y = self.ball_pos.y
        q = quaternion_from_euler(0, 0, 0)
        new_tf.transform.rotation.x = q[0]
        new_tf.transform.rotation.y = q[1]
        new_tf.transform.rotation.z = q[2]
        new_tf.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(new_tf)

    def perform(self):
        self.publish_robots()
        self.publish_ball()
        
        for name in self.robots_name:
            try:
                tf_stamped = self.tf_buffer.lookup_transform(name, "map", rospy.Time(0))
            except Exception as e:
                rospy.logwarn(e)
                return
            trans = tf_stamped.transform.translation
            rot = tf_stamped.transform.rotation
            x = trans.x
            y = trans.y
            theta = euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])[2]
            rospy.loginfo("{} is at x:{}, y:{}, theta:{}".format(name, x, y, theta))
        try:
            tf_stamped = self.tf_buffer.lookup_transform("ball", "map", rospy.Time(0))
        except Exception as e:
            rospy.logwarn(e)
            return
        trans = tf_stamped.transform.translation
        x = trans.x
        y = trans.y
        rospy.loginfo("ball is at x:{}, y:{}".format(x, y))
        

def main():
    rospy.init_node("environment")
    env = environment()
    '''
    for i in range(4):
        env.add_robot('robot'+str(i+1))
    '''
    env.add_robot('base_footprint')
    env.add_ball()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        env.perform()
        rate.sleep()

if __name__ == "__main__":
    main()
