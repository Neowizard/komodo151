#!/usr/bin/env python
import rospy
from cupinator.msg import CtrlCommand, CtrlStatus

"""
Allows users to control the cupinator from the commandline.
"""

# The loop termination var
go = True

# Our ros connection
global publisher
publisher = None
last_robot_state = None

art = """
  +-+         ____            _             _
  | |        / ___|   _ _ __ (_)_ __   __ _| |_  ___  _ __
  | +--+    | |  | | | | '_ \| | '_ \ / _` | __|/ _ \| '__|
  | |       | |__| |_| | |_) | | | | | (_| | |_| (_) | |
  | |        \____\__,_| .__/|_|_| |_|\__,_|\__|\___/|_|
  | |                  |_|
+-+-+----+
|**    **|  Topics in AI, BGU, Fall 2015
*  *--*  *
*  *  *  *   Maor, Lior, Michael
 **    **
"""


def send_command(command_string):
    print "- sending command %s" % command_string
    publisher.publish(CtrlCommand(None, command_string))


def scan():
    send_command("scan")


def stop():
    send_command("stop")


def status():
    print "Last robot status:\n%s" % last_robot_state


def show_help():
    print "Type a command at the prompt."
    print "Available commands are: scan, stop, status, help, quit"


def stop_loop():
    global go
    print "Goodbye!"
    go = False


def ctrl_status_update(message):
    global last_robot_state
    last_robot_state = "%s: %s" % (message.type, message.data)

command_dict = {
    "scan": scan,
    "stop": stop,
    "status": status,
    "help": show_help,
    "quit": stop_loop
}

if __name__ == '__main__':
    print art
    print

    print "Connecting to ROS...",
    rospy.init_node("CupinatorCli", anonymous=True)
    publisher = rospy.Publisher("/cupinator/ctrl/command", CtrlCommand, queue_size=5)
    rospy.Subscriber("/cupinator/ctrl/status", CtrlStatus, ctrl_status_update)

    if publisher is not None:
        print "OK"

    cmd = ""
    while go:
        cmd = raw_input("cupinator> ")
        if cmd in command_dict:
            command_dict[cmd]()
        else:
            print "/!\ unknown command '%s'" % cmd

        print


