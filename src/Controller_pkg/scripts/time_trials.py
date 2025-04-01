#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

# Configuration
TEAM_NAME = "Ctrl-Alt-Defeat"  # Replace with your actual team name
PASSWORD = "Winners"   # Replace with your team password
LINEAR_SPEED = 0.3  # Robot's speed in m/s (adjust as needed)
DISTANCE_LIMIT = 2.0  # Move 1 meter forward
TIMER_TOPIC = "/score_tracker"
CMD_VEL_TOPIC = "/B1/cmd_vel"
CLOCK_TOPIC = "/clock"

# Global variables
start_time = None
elapsed_time = 0
distance_traveled = 0


def start_timer(pub):
    """Start the timer by publishing a message to /score_tracker."""
    global start_time
    start_time = rospy.Time.now()  # Store start time
    start_msg = String(f"{TEAM_NAME},{PASSWORD},0,NA")
    pub.publish(start_msg)
    rospy.loginfo("Timer started!")


def stop_timer(pub):
    """Stop the timer by publishing a message to /score_tracker."""
    stop_msg = String(f"{TEAM_NAME},{PASSWORD},-1,NA")
    pub.publish(stop_msg)
    rospy.loginfo("Timer stopped!")


def move_robot(pub_cmd_vel):
    """Moves the robot forward for 1 meter and then stops."""
    global distance_traveled
    move_cmd = Twist()
    move_cmd.linear.x = LINEAR_SPEED  # Move forward

    rate = rospy.Rate(10)  # 10 Hz control loop
    start_move_time = rospy.Time.now()

    while not rospy.is_shutdown():
        elapsed_time = (rospy.Time.now() - start_move_time).to_sec()
        distance_traveled = LINEAR_SPEED * elapsed_time

        rospy.loginfo(f"Moving... Distance traveled: {distance_traveled:.2f} m")

        if distance_traveled >= DISTANCE_LIMIT:
            rospy.loginfo("Target distance reached. Stopping the robot.")
            move_cmd.linear.x = 0.0  # Stop the robot
            pub_cmd_vel.publish(move_cmd)
            return

        pub_cmd_vel.publish(move_cmd)
        rate.sleep()


def clock_callback(msg):
    """Updates the elapsed time in the Score Tracker."""
    if start_time is not None:
        global elapsed_time
        elapsed_time = (msg.clock - start_time).to_sec()
        rospy.loginfo(f"Elapsed Time: {elapsed_time:.2f} seconds")


def main():
    rospy.init_node('time_trials', anonymous=True)

    # Publishers
    pub_score = rospy.Publisher(TIMER_TOPIC, String, queue_size=1)
    pub_cmd_vel = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)

    # Subscribe to clock to update elapsed time in Score Tracker
    rospy.Subscriber(CLOCK_TOPIC, Clock, clock_callback)

    rospy.sleep(1.0)  # Allow time for ROS setup

    # **Start the Timer**
    start_timer(pub_score)

    # **Move Robot 1 Meter**
    move_robot(pub_cmd_vel)

    # **Stop the Timer**
    stop_timer(pub_score)

    rospy.loginfo("Time Trials Complete!")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
