#!/usr/bin/env python

import rospy
from numpy.random import randn 

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros

from simulator.vision import Vision
# not standard will replace this soon
from simulator.Pos2D import Pos2D

# The major concept needed to change is that the Robot does not cache the change of the pos of the ball, since the black board already cache the ball position. And in real time, it is the blackboard, or say the DSD to publish the positionInfo message
class Robot:
    def __init__(self, name):
        self._name = name
        # self._pos stores the robot's position with noise
        self._pos = Pos2D()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # but cache the publisher
        # equal to the self_localization module
        # publish the position with noise to topic /amcl_pose
        self.pos_pub = rospy.Publisher("/amcl_pose", PoseWithCovarianceStamped, queue_size = 2)

        self.vision = Vision()

    def perform(self):
        self.find_pos()
        self.vision.perform(self._name)
        self.pub_pos()
    
    def find_pos(self):
        try:
            tf_stamped = self.tf_buffer.lookup_transform("map", self._name, rospy.Time(0))
        except Exception as e:
            rospy.logwarn(e)
            return
        trans = tf_stamped.transform.translation
        rot = tf_stamped.transform.rotation
        self._pos.x = trans.x
        self._pos.y = trans.y
        self._pos.theta = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2]
        rospy.loginfo("{} is at x:{}, y:{}, theta:{}".format(self._name, self._pos.x, self._pos.y, self._pos.theta))

    def pub_pos(self):
        # this funciton publishes the robot's position with noise, since self._pos is with noise itself
        new_pos = PoseWithCovarianceStamped()
        # although I think it is important to fill all the blank in new_pos, but according to world_model_capsule, just fill 
        # new_pos.header, 
        # new_pos.pose.pose.x, 
        # new_pos.pose.pose.y, 
        # new_pos,pose.pose.orientation is ok
        new_pos.header.frame_id = "map"
        new_pos.header.stamp = rospy.Time.now()
        new_pos.pose.pose.position.x = self._pos.x
        new_pos.pose.pose.position.y = self._pos.y
        new_orient = quaternion_from_euler(0, 0, self._pos.theta)
        new_pos.pose.pose.orientation.x = new_orient[0]
        new_pos.pose.pose.orientation.y = new_orient[1]
        new_pos.pose.pose.orientation.z = new_orient[2]
        new_pos.pose.pose.orientation.w = new_orient[3]
        self.pos_pub.publish(new_pos)
    '''
    def add_random(self, num):
        return num+float(randn(1))
    '''

def main():
    rospy.init_node('robot')
    robot = Robot('base_footprint')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        robot.perform()
        rate.sleep()
if __name__ == "__main__":
    main()
