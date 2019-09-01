#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8, Bool
from humanoid_league_msgs.msg import GameState

class Referee:
    def __init__(self):
        self.gamestate_pub = rospy.Publisher("gamestate", GameState, queue_size = 2)
        rospy.Subscriber("referee_box_gameState", Int8 , self.referee_box_gameState_callback)
        rospy.Subscriber("referee_box_allowedToMove", Bool, self.referee_box_allowedToMove_callbak)
        self.gs_index = 0
        self.allowedToMove = False

    def perform(self):
        new_gs = GameState()
        new_gs.gameState = self.gs_index
        new_gs.allowedToMove = self.allowedToMove
        rospy.loginfo('CURRENT GAMESTATE is {}'.format(self.gs_index))
        allow_str = 'ALLOWED' if self.allowedToMove else 'NOT ALLOWED'
        rospy.loginfo('now {} to move'.format(allow_str))
        self.gamestate_pub.publish(new_gs)

    def referee_box_gameState_callback(self, gameState):
        self.gs_index = gameState.data

    def referee_box_allowedToMove_callbak(self, allowedToMove):
        self.allowedToMove = allowedToMove.data
        
def main():
    rospy.init_node("Referee")
    referee = Referee()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        referee.perform()
        rate.sleep()

if __name__ == "__main__":
    main()
