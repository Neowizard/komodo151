#!/usr/bin/env python
import rospy
from cupinator.msg import *

global walk_instance


class WalkSim:
    """Logic of the walker module simulator."""

    def __init__(self):
        self.publisher = rospy.Publisher("/cupinator/walk/result", WalkResult, queue_size=5)
        self.first_scan = True

    def walk(self):
        rospy.sleep(2.)
        self.publisher.publish(WalkResult(None, True))


def callback(walk_msg):
    print "Got walk message: ", repr(walk_msg)
    walk_instance.walk()


if __name__ == "__main__":
    global walk_instance
    print "Stating simulation of Walk Module...",

    rospy.init_node("WalkSim", anonymous=False)

    walk_instance = WalkSim()
    rospy.Subscriber("/cupinator/walk/command", WalkCommand, callback)

    print "started."

    rospy.spin()