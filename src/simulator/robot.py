#!/usr/bin/env python

import rospy
import json
from numpy.random import randn 

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

from humanoid_league_msgs.msg import Position2D
from simulator.vision import Vision

# The major concept needed to change is that the Robot does not cache the change of the pos of the ball, since the black board already cache the ball position. And in real time, it is the blackboard, or say the DSD to publish the positionInfo message
class Robot:
    def __init__(self, name):
        self._name = name

        # WARN the robot_pos stores the ground truth of the robot position
        self.robot_pos = Position2D()
        
        # no need to cache the subscriber
        rospy.Subscriber("/robots_pos", String, self.rpos_callback)

        # but cache the publisher
        # publish the position with noise to topic /amcl_pose
        self.pos_pub = rospy.Publisher("/amcl_pose", PoseWithCovarianceStamped, queue_size = 2)

        self.vision = Vision()

    def perform(self):
        self.vision.perform(self.robot_pos)
        self.pub_pose_with_noise()

    def pub_pose_with_noise(self):
        new_pos = PoseWithCovarianceStamped()
        # although I think it is important to fill all the blank in new_pos, but according to world_model_capsule, just fill 
        # new_pos.header, 
        # new_pos.pose.pose.x, 
        # new_pos.pose.pose.y, 
        # new_pos,pose.pose.orientation is ok
        new_pos.header.frame_id = "map"
        new_pos.header.stamp = rospy.Time.now()
        new_pos.pose.pose.position.x = self.add_random(self.robot_pos.pose.x)
        new_pos.pose.pose.position.y = self.add_random(self.robot_pos.pose.y)
        new_orient = quaternion_from_euler(0, 0, self.add_random(self.robot_pos.pose.theta))
        new_pos.pose.pose.orientation.x = new_orient[0]
        new_pos.pose.pose.orientation.y = new_orient[1]
        new_pos.pose.pose.orientation.z = new_orient[2]
        new_pos.pose.pose.orientation.w = new_orient[3]
        self.pos_pub(new_pos)
    
    def rpos_callback(self, inJson):
        inJson = inJson.data
        rpos_msg = json.loads(inJson)
        self.robot_pos.header.frame_id = rpos_msg['frame_id'].encode("utf-8")
        secs = rpos_msg['stamp']['secs']
        nsecs = rpos_msg['stamp']['nsecs']
        self.robot_pos.header.stamp = rospy.Time(secs, nsecs)
        self.robot_pos.pose.x = rpos_msg[self._name]['x'] 
        self.robot_pos.pose.y = rpos_msg[self._name]['y'] 
        self.robot_pos.pose.theta = rpos_msg[self._name]['t'] 
        self.robot_pos.confidence = rpos_msg[self._name]['c']
        rospy.loginfo("{} received x: {}, y: {}, theta{}".format(self._name, rpos_msg[self._name]['x'], rpos_msg[self._name]['y'], rpos_msg[self._name]['t']))
    
    """
    def bpos_callback(self, inJson):
        inJson = inJson.data
        bpos_msg = json.loads(inJson)
        self.ball_pos.header.frame_id = bpos_msg['frame_id'].encode("utf-8")
        secs = bpos_msg['stamp']['secs']
        nsecs = bpos_msg['stamp']['nsecs']
        self.ball_pos.header.stamp = rospy.Time(secs, nsecs)
        self.ball_pos.ball_relative.x = bpos_msg['ball']['x'] 
        self.ball_pos.ball_relative.y = bpos_msg['ball']['y'] 
        self.ball_pos.confidence = bpos_msg['ball']['c'] 
    """

    def add_random(self, num):
        return num+float(randn(1))

def main():
    rospy.init_node('robot1')
    robot = Robot('robot1')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        robot.perform()
        rate.sleep()
if __name__ == "__main__":
    main()
