#!/usr/bin/env python

import rospy
from humanoid_league_msgs.msg import GameState

class GameStateChanger:
    def __init__(self):
        self.gamestate_pub = rospy.Publisher("gamestate", GameState, queue_size = 2)
        self.gs_index = 0
        self.keep_playing = rospy.get_param('/changer/keep_playing')
    def perform(self):
        new_gs = GameState()
        if self.keep_playing:
            new_gs.gameState = 3
        else:
            new_gs.gameState = self.gs_index
        new_gs.allowedToMove = True
        rospy.loginfo('CURRENT GAMESTATE is {}'.format(self.gs_index))
        self.gamestate_pub.publish(new_gs)
        self.gs_index = (self.gs_index + 1) % 5

def main():
    rospy.init_node("GameStateChanger")
    gs_changer = GameStateChanger()
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        gs_changer.perform()
        rate.sleep()

if __name__ == "__main__":
    main()

