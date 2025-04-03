#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler
import time

class AutoMover:
    def __init__(self):
        rospy.init_node("auto_robot_mover")

        self.pub = rospy.Publisher("/B1/cmd_vel", Twist, queue_size=10)
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        # Define teleport locations [(x, y, yaw), ...]
        self.teleport_sequence = [
            (0.488507, 0.035936, 1.572404),
            (-3.969438, 0.476576, -2.615260),
            (-3.974335, -2.278318, -0.018374),
        ]

    def move(self, linear=0.0, angular=0.0, duration=2.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        end_time = rospy.Time.now() + rospy.Duration(duration)
        rospy.loginfo(f"🚙 Moving: linear={linear}, angular={angular}, duration={duration}s")
        while rospy.Time.now() < end_time:
            self.pub.publish(twist)
            rospy.sleep(0.1)

        self.pub.publish(Twist())  # Stop after move

    def teleport(self, x, y, yaw):
        quat = quaternion_from_euler(0.0, 0.0, yaw)
        state = ModelState()
        state.model_name = "B1"
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = 0.2
        state.pose.orientation.x = quat[0]
        state.pose.orientation.y = quat[1]
        state.pose.orientation.z = quat[2]
        state.pose.orientation.w = quat[3]
        state.reference_frame = "world"

        try:
            result = self.set_state(state)
            rospy.loginfo(f"📍 Teleported to x={x}, y={y}, yaw={yaw} → Success={result.success}")
            rospy.sleep(1.0)
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ Teleport failed: {e}")

    def run_sequence(self):
        rospy.sleep(2.0)  # Let sim settle

        # 🔹 Initial move: Drive forward twice with a pause
        self.move(linear=1.0, duration=1.5)
        rospy.sleep(1.0)
        self.move(linear=1.0, duration=8.0)
        rospy.sleep(2.0)

        # === Location 1 ===
        self.teleport(0.488507, 0.035936, 1.572404)
        self.move(linear=-1.0, duration=2.5)
        rospy.sleep(1.0)

        # === Location 2 ===
        self.teleport(-3.969438, 0.476576, -2.2)
        self.move(linear=-1.0, duration=3.5)
        rospy.sleep(1.0)
        self.move(angular=-1.5, duration=1.5)  # rotate in place
        rospy.sleep(2.0)

        # === Location 3 ===
        self.teleport(-3.974335, -2.278318, -0.018374)
        self.move(linear=-1.0, duration=0.5)
        rospy.sleep(1.5)

        rospy.loginfo("✅ Sequence complete.")

if __name__ == "__main__":
    try:
        mover = AutoMover()
        mover.run_sequence()
    except rospy.ROSInterruptException:
        pass
