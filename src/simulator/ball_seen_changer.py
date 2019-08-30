#!/usr/bin/env python

import rospy
import random

from humanoid_league_msgs.msg import BallRelative
from geometry_msgs.msg import Point

class BallSeenChanger:
    def __init__(self):
        self.ballseen_pub = rospy.Publisher("ball_relative", BallRelative, queue_size = 2)
        self.ball_confidence = 0.0
        self.ball_x = 0.0
        self.ball_y = 0.0
        self.ball_z = 0.0

    def perform(self):
        new_ball = BallRelative()
        new_ball.header.frame_id = 'base_footprint'
        new_ball.header.stamp = rospy.Time.now()
        new_ball.ball_relative.x = 0.5 * (self.ball_x + random.random())
        new_ball.ball_relative.y = 0.2 * (self.ball_y + random.random())
        new_ball.ball_relative.z = self.ball_z + random.random()
        _c = random.random()
        if _c > 0.3:
            new_ball.confidence = self.ball_confidence + _c
        else:
            new_ball.confidence = 0
        rospy.loginfo('NOW the ball is at ({}, {}, {}) with confidence({})'.format(new_ball.ball_relative.x, new_ball.ball_relative.y, new_ball.ball_relative.z, new_ball.confidence))
        self.ballseen_pub.publish(new_ball)
def main():
    rospy.init_node("BallSeenChanger")
    bs_changer = BallSeenChanger()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        bs_changer.perform()
        rate.sleep()

if __name__ == "__main__":
    main()

