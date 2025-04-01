#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickControl:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('joystick_control')

        # Publisher for robot velocity commands
        self.cmd_vel_pub = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=10)

        # Subscriber to the joystick input
        self.joy_sub = rospy.Subscriber('/joy_orig', Joy, self.joy_callback)

        # Scaling factors for motion tuning
        self.linear_scale = rospy.get_param("~linear_scale", 0.5)    # meters/sec
        self.angular_scale = rospy.get_param("~angular_scale", 1.0)  # radians/sec

    def joy_callback(self, joy_msg):
        """
        Callback function for Joy messages. Converts joystick inputs to velocity commands.
        :param joy_msg: sensor_msgs/Joy
        """
        twist = Twist()

        # Map joystick axes to robot velocities
        # axes[1] = left stick vertical (forward/backward) -> linear x
        # axes[0] = left stick horizontal (left/right)     -> angular z
        twist.linear.x = self.linear_scale * joy_msg.axes[1]
        twist.angular.z = self.angular_scale * joy_msg.axes[0]

        # Publish the velocity command
        self.cmd_vel_pub.publish(twist)

    def run(self):
        """
        Spins the node, waiting for callbacks.
        """
        rospy.loginfo("Joystick control node started.")
        rospy.spin()


if __name__ == '__main__':
    try:
        joystick_control = JoystickControl()
        joystick_control.run()
    except rospy.ROSInterruptException:
        pass

