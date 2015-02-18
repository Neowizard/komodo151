#!/usr/bin/env python
import rospy
from cupinator.msg import CtrlCommand

"""
Main control node for the Cupinator. Talks to the user via a topic, and maintains a state machine.

"""


def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)

"""
States of the robot's state machine
"""
States = enum("IDLE", "SCAN", "GO_TO_CUP", "AT_CUP", "NO_CUP")

currentState = States.IDLE


def callback(command_msg):
    rospy.loginfo("got message %s", command_msg.command)


if __name__ == '__main__':
    rospy.loginfo("Starting the controller node")
    rospy.init_node("CupinatorCtrl", anonymous=False)
    rospy.Subscriber("/cupinator/ctrl/command", CtrlCommand, callback)
    rospy.spin()



