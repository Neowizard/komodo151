#!/usr/bin/env python
import rospy
from cupinator.msg import *

global walk_instance


class RoomScannerSim:
    """Logic of the room scanner simulator."""

    def __init__(self, initial_distance):
        self.distance = initial_distance
        self.publisher = rospy.Publisher("/cupinator/room_scanner/result", RoomScannerResult, queue_size=5)
        self.first_scan = True
        rospy.Subscriber("/cupinator/room_scanner/command", RoomScannerCommand, self.scan_room)

    def scan_room(self, cmd):
        print "Got scan message: ", repr(cmd)
        cur_dist = self.distance
        self.distance /= 2
        cur_dist = 0 if cur_dist < 10 else cur_dist
        rospy.sleep(1.)
        self.publisher.publish(RoomScannerResult(None, 10.0, 5, cur_dist))


if __name__ == "__main__":
    global walk_instance
    print "Stating simulation of Room Scanner...",

    rospy.init_node("RoomScannerSim", anonymous=False)
    walk_instance = RoomScannerSim(1000)

    print "started."

    rospy.spin()