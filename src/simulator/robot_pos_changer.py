#!/usr/bin/env python

import rospy
import random
import json

from std_msgs.msg import String

class RobotPosChanger:
    def __init__(self, name):
        self.robotpos_pub = rospy.Publisher(name+"/pos", String, queue_size = 2)
        self._name = name
        self.pos = dict()
        self.pos['x'] = 0
        self.pos['y'] = 0
        self.ballpos= dict()
        self.ballpos['x'] = 0
        self.ballpos['y'] = 0
    
    def perform(self):
        self.pos['x'] = random.random()
        self.pos['y'] = random.random()
        self.ballpos['x'] = random.random()
        self.ballpos['y'] = random.random()
        outDict = {'name': self._name, 'pos': self.pos, 'ballpos': self.ballpos}
        outJson = json.dumps(outDict)
        rospy.loginfo("{}'s position is (x: {}, y: {})".format(self._name, self.pos['x'], self.pos['y']))
        rospy.loginfo("{}'s BALL position is (x: {}, y: {})".format(self._name, self.ballpos['x'], self.ballpos['y']))
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
