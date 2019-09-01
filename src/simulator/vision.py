#!/usr/bin/env python

# get ground truth robot position from robot it self
# and the ground truth ball position from topic '/robots_pos'

import rospy
from math import atan2 # be sure to use as atan2(y, x)
from math import pi as PI
import random

import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String

from humanoid_league_msgs.msg import BallRelative

class Vision:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        #this bpos_pub publishes whether the robot think it sees the ball and the relative position of the ball
        self.bpos_pub = rospy.Publisher('/ball_relative', BallRelative, queue_size = 2)

    # no need to pass bpos to perform. get it from cached_ballpos
    def perform(self, name):
        try:
            tf_stamped = self.tf_buffer.lookup_transform("ball", name, rospy.Time(0))
        except Exception as e:
            rospy.logwarn(e)
            return
        trans = tf_stamped.transform.translation
        rot = tf_stamped.transform.rotation
        delta_x = trans.x
        delta_y = trans.y
        # after restrict_angle(), delta_theta is in [0, 2*PI]
        delta_theta = self.restrict_angle(atan2(delta_y, delta_x))
        # according to euler_from_quaternion, robo_dir_theta now is between[-PI, PI]
        robot_dir_theta = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2]
        # after restrict_angle(), robot_dir_theta is in [0, 2*PI]
        robot_dir_theta = self.restrict_angle(robot_dir_theta)
        # thus, (delta_theta - robot_dir_theta) is in [-2*PI, 2*PI]
        # after restrict_angle(), delta_angle is in [0, 2*PI]
        delta_angle = delta_theta - robot_dir_theta
        delta_angle = self.restrict_angle(delta_angle)
        condition1 = 0 < delta_angle < 1/4*PI
        condition2 = 7/4*PI < delta_angle < 2*PI
        rospy.loginfo("condition1 is {}, condition2 is {}".format(condition1, condition2))
        if condition1 or condition2:
            rospy.loginfo("VISION sees the ball at relative position x:{}, y:{}".format(delta_x, delta_y))
            new_bpos = BallRelative()
            new_bpos.header.frame_id = "base_footprint"
            new_bpos.header.stamp = rospy.Time.now()
            new_bpos.ball_relative.x = delta_x
            new_bpos.ball_relative.y = delta_y
            new_bpos.confidence = random.random()
            self.bpos_pub.publish(new_bpos)
        else:
            rospy.loginfo("no ball seen")

    def ballpos_callback(self, inJson):
        inJson = inJson.data
        bpos_msg = json.loads(inJson)
        self.cached_ballpos.header.frame_id = bpos_msg['frame_id'].encode("utf-8")
        secs = bpos_msg['stamp']['secs']
        nsecs = bpos_msg['stamp']['nsecs']
        self.cached_ballpos.header.stamp = rospy.Time(secs, nsecs)
        self.cached_ballpos.pose.x = bpos_msg['ball']['x']
        self.cached_ballpos.pose.y = bpos_msg['ball']['y']
        self.cached_ballpos.confidence = bpos_msg['ball']['c']

    def restrict_angle(self, angle):
        return angle % (2 * PI)
