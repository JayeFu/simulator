#!/usr/bin/env python

import rospy
import random
import json
from math import pi as PI

from std_msgs.msg import String

class RobotPosChanger:
    def __init__(self, name):
        self.robotpos_pub = rospy.Publisher(name+"/positionInfo", String, queue_size = 2)
        self._name = name
        self.pos = dict()
        self.pos['x'] = 0
        self.pos['y'] = 0
        self.pos['t'] = 0
        # r indicates relative ^_^
        self.r_ballpos= dict()
        self.r_ballpos['x'] = 0
        self.r_ballpos['y'] = 0
    
    def perform(self):
        self.pos['x'] = 9 * random.random() - 4.5
        self.pos['y'] = 6 * random.random() - 3.0
        self.pos['t'] = 2 * PI *random.random()
        self.r_ballpos['x'] = random.random()
        self.r_ballpos['y'] = random.random()
        outDict = {'name': self._name, 'pos': self.pos, 'ballpos': self.r_ballpos}
        outJson = json.dumps(outDict)
        rospy.loginfo("{}'s position is (x: {}, y: {}, theta: {})".format(self._name, self.pos['x'], self.pos['y'], self.pos['t']))
        rospy.loginfo("{}'s seen BALL position is (x: {}, y: {})".format(self._name, self.r_ballpos['x'], self.r_ballpos['y']))
        self.robotpos_pub.publish(outJson)

def main():
    rospy.init_node("RobotPosChanger")
    node_name = rospy.get_name()
    robot_name = rospy.get_param(node_name+"/name")
    # robot_name = "robot1"
    rp_changer = RobotPosChanger(robot_name)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        rp_changer.perform()
        rate.sleep()

if __name__ == "__main__":
    main()
