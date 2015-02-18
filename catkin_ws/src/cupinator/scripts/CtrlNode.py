#!/usr/bin/env python
import rospy
from cupinator.msg import *

"""
Main control node for the Cupinator. Talks to the user via a topic, and maintains a state machine.

States:
    IDLE, ROOM_SCAN, MOVING_TO_CUP, AT_CUP, NO_CUP

State Machine:
search
    - found-> go-to-cup
    - not-found -> no-cup
go-to-cup
    - got cup
    - obstacle
"""

# Global variables
global current_state
global status_publisher
global room_scanner_publisher
status_publisher = None
current_state = None
room_scanner_publisher = None


def change_state(new_state):
    """
    Changes the state of the node, and publishes it over the topic.
    :param new_state:   new stat the robot is in
    :return: None
    """
    global current_state
    current_state = new_state
    status_publisher.publish(CtrlStatus(None, "state_change", new_state))


def publish_error(message):
    """
    publishes an error message to the status topic
    """
    status_publisher.publish(CtrlStatus(None, "error", message))


def start_scan():
    """ Starts the search for the cup """
    if current_state not in ("IDLE", "NO_CUP"):
        publish_error("Cannot initiate room scan from state '%s'" % current_state)
    else:
        change_state("ROOM_SCAN")
        room_scanner_publisher.publish(RoomScannerCommand(None, "start", True))


def scan_callback(room_scanner_result):
    rospy.loginfo("Got scanner result: %s" % repr(room_scanner_result))
    change_state("IDLE")
# TODO continue here - decide whether to go to the best place of not.


def stop():
    change_state("IDLE")
# TODO send stop message to the proper node, if any.

def command_callback(command_msg):
    command = command_msg.command
    rospy.loginfo("command %s", command)
    if command == "scan":
        start_scan()
    elif command == "stop":
        stop()
    else:
        publish_error("bad command: '%s'" % command)


if __name__ == '__main__':
    try:
        rospy.loginfo("Starting the controller node")
        rospy.init_node("CupinatorCtrl", anonymous=False)

        status_publisher = rospy.Publisher("/cupinator/ctrl/status", CtrlStatus, queue_size=10)
        rospy.Subscriber("/cupinator/ctrl/command", CtrlCommand, command_callback)

        room_scanner_publisher = rospy.Publisher("/cupinator/room_scanner/command", RoomScannerCommand, queue_size=5)
        rospy.Subscriber("/cupinator/room_scanner/result", RoomScannerResult, scan_callback)

        change_state("IDLE")

        rospy.spin()
    except rospy.ROSInterruptException:
        pass




