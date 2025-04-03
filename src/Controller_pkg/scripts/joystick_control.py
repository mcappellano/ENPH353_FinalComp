#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler
import subprocess
import os
import signal
import time


def launch_terminal_at_position(session_name, x=1500, y=800):
    """Launch gnome-terminal and move it to a specific screen position."""
    subprocess.Popen([
        "gnome-terminal", "--title", session_name, "--", "bash", "-c",
        f"tmux attach-session -t {session_name}; exec bash"
    ])
    time.sleep(0.3)
    subprocess.call([
        "wmctrl", "-r", session_name, "-e", f"0,{x},{y},900,300"
    ])


class JoystickToCmdVel:
    def __init__(self):
        rospy.init_node("joystick_control_node", anonymous=True)

        self.cmd_pub = rospy.Publisher("/B1/cmd_vel", Twist, queue_size=10)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)

        self.linear_axis = rospy.get_param("~linear_axis", 1)
        self.angular_axis = rospy.get_param("~angular_axis", 3)
        self.linear_scale = rospy.get_param("~linear_scale", 2.0)
        self.angular_scale = rospy.get_param("~angular_scale", 4.0)

        self.tracking_active = False
        self.last_trigger_state = 0
        self.trigger_button_index = 5  # RB
        self.reset_button_index = 4    # LB
        self.last_reset_state = 0

        self.model_button_index = 0  # X button on PS4
        self.last_model_button_state = 0
        self.model_active = False
        self.model_process = None

        self.contingency_button_index = 3  # Y / Triangle button
        self.last_contingency_state = 0
        self.contingency_active = False
        self.contingency_process = None


        rospy.loginfo("Joystick control node initialized.")
        rospy.spin()

    def joy_callback(self, msg):
        # Check for model toggle (X button)
        model_button = msg.buttons[self.model_button_index]
        if model_button == 1 and self.last_model_button_state == 0:
            self.toggle_model_node()
        self.last_model_button_state = model_button

        # Ignore joystick motion while model is active
        if self.model_active:
            return

        # Normal teleop
        linear = msg.axes[self.linear_axis] * self.linear_scale
        angular = msg.axes[self.angular_axis] * self.angular_scale

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

        # Image tracking toggle (R2)
        trigger_pressed = msg.buttons[self.trigger_button_index]
        if trigger_pressed == 1 and self.last_trigger_state == 0:
            self.toggle_tracking()
        self.last_trigger_state = trigger_pressed

        # Robot reset (Triangle)
        reset_pressed = msg.buttons[self.reset_button_index]
        if reset_pressed == 1 and self.last_reset_state == 0:
            self.reset_robot_pose()
        self.last_reset_state = reset_pressed

        # Contingency plan toggle (Y button)
        contingency_pressed = msg.buttons[self.contingency_button_index]
        if contingency_pressed == 1 and self.last_contingency_state == 0:
            self.toggle_contingency_plan()
        self.last_contingency_state = contingency_pressed

    def toggle_tracking(self):
        if not self.tracking_active:
            self.current_session_name = f"image_tracker_{int(time.time())}"

            rospy.loginfo(f"🎥 Starting new image tracking session: {self.current_session_name}")

            subprocess.call(
                f"tmux new-session -d -s {self.current_session_name} "
                f"'cd ~/ros_ws/src/Controller_pkg/scripts && source ~/ros_ws/devel/setup.bash && rosrun Controller_pkg tracker_analog.py'",
                shell=True
            )

            launch_terminal_at_position(self.current_session_name, x=1500, y=800)

            self.tracking_active = True

        else:
            if not hasattr(self, 'current_session_name'):
                rospy.logwarn("⚠️ No previous session name found.")
                self.tracking_active = False
                return

            rospy.loginfo(f"🛑 Stopping session: {self.current_session_name}")
            subprocess.call(["tmux", "send-keys", "-t", self.current_session_name, "C-c"])
            self.tracking_active = False

    def reset_robot_pose(self):
        """Reset the robot 'B1' to its original spawn location in Gazebo."""
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            quat = quaternion_from_euler(0.0, 0.0, -1.57)

            state_msg = ModelState()
            state_msg.model_name = 'B1'
            state_msg.pose.position.x = 5.5
            state_msg.pose.position.y = 2.5
            state_msg.pose.position.z = 0.2
            state_msg.pose.orientation.x = quat[0]
            state_msg.pose.orientation.y = quat[1]
            state_msg.pose.orientation.z = quat[2]
            state_msg.pose.orientation.w = quat[3]
            state_msg.reference_frame = 'world'

            set_state(state_msg)
            rospy.loginfo("🔁 Robot B1 pose reset to initial launch position.")
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ Failed to reset robot pose: {e}")

    def toggle_model_node(self):
        if not self.model_active:
            rospy.loginfo("🧠 Starting run_model_node.py...")
            self.model_process = subprocess.Popen(
                [
                    "bash", "-c",
                    "source ~/ros_ws/devel/setup.bash && rosrun Controller_pkg run_imitation_learning.py"
                ],
                preexec_fn=os.setsid
            )

            self.model_active = True
        else:
            if self.model_process and self.model_process.poll() is None:
                rospy.loginfo("🛑 Stopping run_model_node.py...")
                os.killpg(os.getpgid(self.model_process.pid), signal.SIGINT)
            self.model_active = False

    def toggle_contingency_plan(self):
        if not self.contingency_active:
            rospy.loginfo("📦 Starting contingency_plan.py...")
            self.contingency_process = subprocess.Popen(
                [
                    "bash", "-c",
                    "source ~/ros_ws/devel/setup.bash && rosrun Controller_pkg contingency_plan.py"
                ],
                preexec_fn=os.setsid
            )
            self.contingency_active = True
        else:
            if self.contingency_process and self.contingency_process.poll() is None:
                rospy.loginfo("🛑 Stopping contingency_plan.py...")
                os.killpg(os.getpgid(self.contingency_process.pid), signal.SIGINT)
            self.contingency_active = False


if __name__ == "__main__":
    try:
        JoystickToCmdVel()
    except rospy.ROSInterruptException:
        pass



#

