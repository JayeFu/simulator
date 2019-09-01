# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'referee_box.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_TempWindow(object):
    def setupUi(self, TempWindow):
        TempWindow.setObjectName("TempWindow")
        TempWindow.resize(400, 514)
        self.centralwidget = QtWidgets.QWidget(TempWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.layoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(30, 10, 311, 451))
        self.layoutWidget.setObjectName("layoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.pbt_initial = QtWidgets.QPushButton(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(19)
        self.pbt_initial.setFont(font)
        self.pbt_initial.setObjectName("pbt_initial")
        self.verticalLayout.addWidget(self.pbt_initial)
        self.pbt_ready = QtWidgets.QPushButton(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(19)
        self.pbt_ready.setFont(font)
        self.pbt_ready.setObjectName("pbt_ready")
        self.verticalLayout.addWidget(self.pbt_ready)
        self.pbt_set = QtWidgets.QPushButton(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(19)
        self.pbt_set.setFont(font)
        self.pbt_set.setObjectName("pbt_set")
        self.verticalLayout.addWidget(self.pbt_set)
        self.pbt_playing = QtWidgets.QPushButton(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(19)
        self.pbt_playing.setFont(font)
        self.pbt_playing.setObjectName("pbt_playing")
        self.verticalLayout.addWidget(self.pbt_playing)
        self.pbt_finished = QtWidgets.QPushButton(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(19)
        self.pbt_finished.setFont(font)
        self.pbt_finished.setObjectName("pbt_finished")
        self.verticalLayout.addWidget(self.pbt_finished)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.pbt_allowed_to_move = QtWidgets.QPushButton(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(19)
        self.pbt_allowed_to_move.setFont(font)
        self.pbt_allowed_to_move.setObjectName("pbt_allowed_to_move")
        self.horizontalLayout.addWidget(self.pbt_allowed_to_move)
        TempWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(TempWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 400, 22))
        self.menubar.setObjectName("menubar")
        TempWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(TempWindow)
        self.statusbar.setObjectName("statusbar")
        TempWindow.setStatusBar(self.statusbar)

        self.retranslateUi(TempWindow)
        QtCore.QMetaObject.connectSlotsByName(TempWindow)

    def retranslateUi(self, TempWindow):
        _translate = QtCore.QCoreApplication.translate
        TempWindow.setWindowTitle(_translate("TempWindow", "Referee Box"))
        self.pbt_initial.setText(_translate("TempWindow", "INITIAL"))
        self.pbt_ready.setText(_translate("TempWindow", "READY"))
        self.pbt_set.setText(_translate("TempWindow", "SET"))
        self.pbt_playing.setText(_translate("TempWindow", "PLAYING"))
        self.pbt_finished.setText(_translate("TempWindow", "FINISHED"))
        self.pbt_allowed_to_move.setText(_translate("TempWindow", "MOVE"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    TempWindow = QtWidgets.QMainWindow()
    ui = Ui_TempWindow()
    ui.setupUi(TempWindow)
    TempWindow.show()
    sys.exit(app.exec_())

