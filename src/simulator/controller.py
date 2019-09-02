#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
import tf2_ros
from tf.transformations import euler_from_quaternion

from humanoid_league_msgs.msg import RobotControlState, Position2D

class Controller:
    def __init__(self):
        self.position = Position2D()

        self.robot_state_pub = rospy.Publisher("robot_state", RobotControlState, queue_size = 2)

        self.vel_pub = rospy.Publisher("robot/cmd_vel", Twist, queue_size=1)

        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.move_base_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.position_callback)

    def publish_robot_state(self, state):
        new_robot_state = RobotControlState()
        new_robot_state.state = state
        self.robot_state_pub.publish(new_robot_state)

    def move_base_callback(self, pos):
        msg = Twist()
        delta_x = pos.pose.position.x - self.position.pose.x
        delta_y = pos.pose.position.y - self.position.pose.y
        # the following coefficient from learning_tf2.turtle_tf2_listner
        msg.angular.z = 4 * math.atan2(delta_y, delta_x)
        msg.linear.x = 0.5 * math.sqrt(delta_x**2 + delta_y**2)
        self.vel_pub.publish(msg)

    def position_callback(self, pos):
        position2d = Position2D()
        position2d.header = pos.header
        position2d.pose.x = pos.pose.pose.position.x
        position2d.pose.y = pos.pose.pose.position.y
        rotation = pos.pose.pose.orientation
        position2d.pose.theta = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])[2]
        self.position = position2d

def main():
    rospy.init_node("controller")
    controller = Controller()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    main()
