#!/usr/bin/env python

import rospy

from simulator.TempWindow import Ui_TempWindow
from std_msgs.msg import Bool, Int8

from humanoid_league_msgs.msg import GameState

class RefereeBox(Ui_TempWindow):
    def __init__(self):
        super(RefereeBox, self).__init__()
        
        rospy.init_node("RefereeBox")

        self.gameState_pub = rospy.Publisher("referee_box_gameState", Int8, queue_size = 2)
        self.allowedToMove_pub = rospy.Publisher("referee_box_allowedToMove", Bool, queue_size = 2)

    def setup_signals_slots(self):
        self.pbt_initial.clicked.connect(lambda: self.publish_gameState(GameState.GAMESTATE_INITIAL))
        self.pbt_ready.clicked.connect(lambda: self.publish_gameState(GameState.GAMESTATE_READY))
        self.pbt_set.clicked.connect(lambda: self.publish_gameState(GameState.GAMESTATE_SET))
        self.pbt_playing.clicked.connect(lambda: self.publish_gameState(GameState.GAMESTATE_PLAYING))
        self.pbt_finished.clicked.connect(lambda: self.publish_gameState(GameState.GAMESTATE_FINISHED))
        self.pbt_allowed_to_move.clicked.connect(lambda: self.publish_allowedToMove(True))


    def publish_gameState(self, gameState):
        self.gameState_pub.publish(gameState)

    def publish_allowedToMove(self, allowedToMove):
        self.allowedToMove_pub.publish(allowedToMove)

def main():
    from PyQt5 import QtWidgets
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = RefereeBox()
    ui.setupUi(MainWindow)
    ui.setup_signals_slots()
    MainWindow.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
