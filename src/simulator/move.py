#!/usr/bin/env python

import rospy

from geometry_msgs.msg import TransformStamped, Twist, Pose2D
from tf2_geometry_msgs import PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros

from simulator.Pos2D import add_noise

# class Move controls the movement of the robot
# should operate a quite high frequency

class Move:
    def __init__(self):
        self.angular_z = 0
        self.linear_x = 0
        self.linear_y = 0
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(2))
        self.tf_listner = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.Subscriber("base_footprint/cmd_vel", Twist, self.cmd_vel_callback)
        self.pos_pub_gt = rospy.Publisher("base_footprint/pos_gt", Pose2D, queue_size=2)

    def cmd_vel_callback(self, msg):
        # the following velocity is under the coordinate of the robot itself
        self.angular_z = add_noise(msg.angular.z)
        self.linear_x = add_noise(msg.linear.x)
        self.linear_y = add_noise(msg.linear.y)
        # although the pose calculated by the robot has been added noise in the process, we still need to add some noise here, since the action implemented by the robot cannot be very accurate.
        # two ways of changing the pos of the robot. One is directly change the pos by tf broadcaster, other is send topic to the environment. Then the environment change the position it self.
        # th new_pos2d should be under
    
    def perform(self, time_interval):
        # first look at the robot's position
        new_ps = PoseStamped()
        new_ps.header.frame_id = "base_footprint_gt"
        new_ps.header.stamp = rospy.Time.now()
        new_ps.pose.position.x = time_interval * self.linear_x
        new_ps.pose.position.y = time_interval * self.linear_y
        delta_theta = time_interval * self.angular_z
        q = quaternion_from_euler(0, 0, delta_theta)
        new_ps.pose.orientation.x = q[0]
        new_ps.pose.orientation.y = q[1]
        new_ps.pose.orientation.z = q[2]
        new_ps.pose.orientation.w = q[3]
        try:
            tfed_ps = self.tf_buffer.transform(new_ps, "map", timeout=rospy.Duration(0.5))
        except Exception as e:
            rospy.logwarn(e)
            return
        new_pose2d = Pose2D()
        new_pose2d.x = tfed_ps.pose.position.x
        new_pose2d.y = tfed_ps.pose.position.y
        q = tfed_ps.pose.orientation
        a = euler_from_quaternion([q.x, q.y, q.z, q.w])
        new_pose2d.theta = a[2]
        self.pos_pub_gt.publish(new_pose2d)
        rospy.loginfo("new pos of the robot is at x:{}, y:{}, theta{}".format(new_pose2d.x, new_pose2d.y, new_pose2d.theta))

def main():
    rospy.init_node("move")
    move = Move()
    frequency = 20.0
    rate = rospy.Rate(frequency)
    time_interval = 1.0/frequency
    # the sample time will be 0.05s
    while not rospy.is_shutdown():
        move.perform(time_interval)

if __name__ == "__main__":
    main()
