#!/usr/bin/env python

import rospy
import random

from humanoid_league_msgs.msg import GoalRelative
from geometry_msgs.msg import Point

class GoalSeenChanger:
    def __init__(self):
        self.goal_seen_pub = rospy.Publisher("goal_relative", GoalRelative, queue_size = 2)
        self.goal_changing = rospy.get_param('/changer/goal_changing')
    def perform(self):
        new_goal = GoalRelative()
        new_goal.header.frame_id = 'odom'
        new_goal.header.stamp = rospy.Time.now()
        if self.goal_changing:
            new_goal.right_post = Point(2+random.random(), 1+random.random(), 0+random.random())
            new_goal.left_post = Point(2+random.random(), 1+random.random(), 0+random.random())
        else:
            new_goal.right_post = Point(2, 1, 0)
            new_goal.left_post = Point(1.5, 0.7, 0.5)
        new_goal.confidence = random.random()
        self.goal_seen_pub.publish(new_goal)

def main():
    rospy.init_node("GoalSeenChanger")
    gs_changer = GoalSeenChanger()
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        gs_changer.perform()
        rate.sleep()

if __name__ == "__main__":
    main()
