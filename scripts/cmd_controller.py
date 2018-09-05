#! /usr/bin/python
"""
A basic ros node written in python that publishes commands typed into
/cmd. Other nodes interested in this command should listen to this topic,
parse the string in the callback and do something. 

Type of the message is a ros std_msgs String

You can expand this to parse and send/publish other things too such as goals,
or twist(drive) commands. 
"""

import rospy
# Note - for debugging you can also do something similar here with goal. 
# Send a PoseStamped into the goal topic. 
# from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String

def send_goal_loop():
    cmd_pub = rospy.Publisher('/cmd', String, queue_size=10)

    while True:

        # Wait for a user to print something. 
        command = raw_input()

        # Clean-ish exit
        if command == '\n':
            break
        elif command == '':
            break
        else: 
            print "Sending command: " + command
            # Publishes the command. 
            cmd_pub.publish(command)

        # publish message here
        rospy.Rate(10).sleep()


if __name__ == "__main__":
    rospy.init_node('command_controller_node')
    send_goal_loop()
