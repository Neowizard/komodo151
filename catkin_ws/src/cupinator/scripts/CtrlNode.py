#!/usr/bin/env python
import rospy
from cupinator.msg import *

"""
Main control node for the Cupinator. Talks to the user via a topic, and maintains a state machine.

States:
    IDLE, ROOM_SCAN, APPROACHING_CUP, ARRIVING_AT_CUP, READJUSTING_BEARING, AT_CUP, NO_CUP, MISSION_FAIL

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
global walk_publisher
status_publisher = None
current_state = None
room_scanner_publisher = None
walk_publisher = None

# TODO move to parameter server?
# Cost above which we conclude there's no cup.
COST_THRESHOLD = 1000
# Distance below which we go straight to the cup (cm)
STRAIGHT_WALK_MIN_DIST = 100

##
# Staring actions on the robot
##


def start_scan():
    """ Starts the search for the cup """
    if current_state not in ("IDLE", "NO_CUP"):
        publish_error("Cannot initiate room scan from state '%s'" % current_state)
    else:
        change_state("ROOM_SCAN")
        room_scanner_publisher.publish(RoomScannerCommand(None, "start", True))


def start_approach(room_scanner_result):
    """
    Start approaching the cup. Above a certain distance, we do this by going half the distance and re-adjusting
    the direction. Below that distance, we go straight.
    :return: None
    """
    angle = room_scanner_result.angle
    distance = room_scanner_result.distance
    if distance > STRAIGHT_WALK_MIN_DIST:
        distance /= 2
        change_state("APPROACHING_CUP")
    else:
        change_state("ARRIVING_AT_CUP")

    walk_publisher.publish(WalkCommand(None, angle, distance))


def start_bearing_readjustment():
    """
    Finished a leg on our way to the cup. Now re-adjust the robot's bearing to still have the cup
    straight ahead.
    """
    change_state("READJUSTING_BEARING")
    room_scanner_publisher.publish(RoomScannerCommand(None, "start", False))


def stop():
    """
    Stop the robot from doing the task, in an orderly manner.
    """
    change_state("IDLE")
# TODO send stop message to the proper node, if any.


def abort():
    """
    Admit that while we did recognize a cup, we couldn't get to it.
    """
    stop()
    change_state("MISSION_FAIL")


##
# Callbacks
##


def scan_callback(room_scanner_result):
    if current_state == "ROOM_SCAN":
        if room_scanner_result.cost > COST_THRESHOLD:
            change_state("NO_CUP")
        else:
            start_approach(room_scanner_result)

    elif current_state == "READJUSTING_BEARING":
        if room_scanner_result.cost > COST_THRESHOLD:
            publish_error("Lost track of cup")
            abort()
        else:
            start_approach(room_scanner_result)


def walk_callback(walk_result):
    if current_state == "APPROACHING_CUP":
        if walk_result.result:
            start_bearing_readjustment()
        else:
            publish_error("Walk module failed to advance.")
            abort()

    elif current_state == "ARRIVING_AT_CUP":
        change_state("AT_CUP")  # Success!

    else:
        publish_error("Got a walk result while not walking. Weird. Ignoring.")


def command_callback(command_msg):
    """
    Got a command from the Cli
    """
    command = command_msg.command
    rospy.loginfo("command %s", command)
    if command == "scan":
        start_scan()
    elif command == "stop":
        stop()
    else:
        publish_error("bad command: '%s'" % command)

##
# Internal methods
##


def change_state(new_state):
    """
    Changes the state of the node, and publishes it over the topic.
    :param new_state:   new stat the robot is in
    :return: None
    """
    global current_state
    current_state = new_state
    rospy.loginfo("current state: %s" % current_state)
    status_publisher.publish(CtrlStatus(None, "state_change", current_state))


def publish_error(message):
    """
    publishes an error message to the status topic
    """
    status_publisher.publish(CtrlStatus(None, "error", message))


if __name__ == '__main__':
    try:
        rospy.loginfo("Starting the controller node")
        rospy.init_node("CupinatorCtrl", anonymous=False)

        status_publisher = rospy.Publisher("/cupinator/ctrl/status", CtrlStatus, queue_size=10)
        rospy.Subscriber("/cupinator/ctrl/command", CtrlCommand, command_callback)

        room_scanner_publisher = rospy.Publisher("/cupinator/room_scanner/command", RoomScannerCommand, queue_size=5)
        rospy.Subscriber("/cupinator/room_scanner/result", RoomScannerResult, scan_callback)
        
        walk_publisher = rospy.Publisher("/cupinator/walk/command", WalkCommand, queue_size=5)
        rospy.Subscriber("/cupinator/walk/result", WalkResult, walk_callback )

        change_state("IDLE")

        rospy.spin()
    except rospy.ROSInterruptException:
        pass




