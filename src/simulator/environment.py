#!/usr/bin/env python

# Provide the infomation of the global position of the ball and the robot, which is definitely right
# The pos of the robot is Position2D, including header(std_msgs/Header-->seq(uint32), stamp(time), frame_id(string)), pose(geometry_msgs/Pose2D-->x, y, theta(all float64)), confidence(float32)
# The pos of the ball is BallRelative, including header(std_msgs/Header), ball_relative(geometry_msgs/Point-->x, y, z(all float64)), confidence(float32)

import rospy
import random
from math import pi as PI
import json

from std_msgs.msg import String

from humanoid_league_msgs.msg import Position2D, BallRelative

class environment:
    def __init__(self):
        self.robots_pos = dict()
        self.ball_pos = BallRelative()
        # both the messages are JSON ecoded msg
        self.rpos_pub = rospy.Publisher("/robots_pos", String, queue_size = 2)
        self.bpos_pub = rospy.Publisher("/ball_pos", String, queue_size = 2)

        # randomlize the ball_pos
        self.ball_pos.header.stamp = rospy.Time.now()
        self.ball_pos.header.frame_id = "field"
        self.ball_pos.ball_relative.x = 9.0 * random.random() - 4.5
        self.ball_pos.ball_relative.y = 6.0 * random.random() - 3.0
        self.ball_pos.confidence = 1.0

    def add_robot(self, name):
        self.robots_pos[name] = Position2D()
        # define rpos to shorten the code
        rpos = self.robots_pos[name]
        # randomlize the robots_pos
        rpos.header.stamp = rospy.Time.now()
        rpos.header.frame_id = "field"
        rpos.pose.x = 9.0 * random.random() - 4.5
        rpos.pose.y = 6.0 * random.random() - 3.0
        rpos.pose.theta = 2 * PI * random.random()
        rpos.confidence = 1.0

    def perform(self):
        rpos_msg = dict()
        bpos_msg = dict()
        now = rospy.Time.now()
        # omit the prefix 'header' for convenience
        rpos_msg['frame_id'] = "field"
        rpos_msg['stamp'] = dict()
        rpos_msg['stamp']['secs'] = now.secs
        rpos_msg['stamp']['nsecs'] = now.nsecs
        for rname, rpos in self.robots_pos.iteritems():
            rpos_msg[rname] = self._Position2D2Dict(self.robots_pos[rname])

        bpos_msg['frame_id'] = "field"
        bpos_msg['stamp'] = dict()
        bpos_msg['stamp']['secs'] = now.secs
        bpos_msg['stamp']['nsecs'] = now.nsecs
        bpos_msg['ball'] = self._BallRelative2Dict(self.ball_pos)

        rpos_msg = json.dumps(rpos_msg)
        bpos_msg = json.dumps(bpos_msg)
        
        self.rpos_pub.publish(rpos_msg)
        self.bpos_pub.publish(bpos_msg)

    def _Position2D2Dict(self, pos):
        new_rpos_dict = dict()
        new_rpos_dict['x'] = pos.pose.x
        new_rpos_dict['y'] = pos.pose.y
        new_rpos_dict['t'] = pos.pose.theta
        new_rpos_dict['c'] = pos.confidence
        return new_rpos_dict
        
    def _BallRelative2Dict(self, pos):
        new_bpos_dict = dict()
        new_bpos_dict['x'] = pos.ball_relative.x
        new_bpos_dict['y'] = pos.ball_relative.y
        new_bpos_dict['c'] = pos.confidence
        return new_bpos_dict

def main():
    rospy.init_node("environment")
    env = environment()
    for i in range(4):
        env.add_robot('robot'+str(i+1))
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        env.perform()
        rate.sleep()

if __name__ == "__main__":
    main()
