#!/usr/bin/env python
import rospy
from cupinator.msg import *

global walk_instance


class WalkSim:
    """Logic of the walker module simulator."""

    def __init__(self):
        self.publisher = rospy.Publisher("/cupinator/walk/result", WalkResult, queue_size=5)
        self.first_scan = True
        rospy.Subscriber("/cupinator/walk/command", WalkCommand, self.walk)

    def walk(self, walk_msg):
        print "Got walk message: ", repr(walk_msg)
        rospy.sleep(2.)
        self.publisher.publish(WalkResult(None, True))


if __name__ == "__main__":
    global walk_instance
    print "Stating simulation of Walk Module...",

    rospy.init_node("WalkSim", anonymous=False)
    walk_instance = WalkSim()

    print "started."

    rospy.spin()