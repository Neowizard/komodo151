#!/usr/bin/env python
import rospy
from cupinator.msg import CtrlCommand

"""
Allows users to control the cupinator from the commandline.
"""

# The loop termination var
go = True

# Our ros connection
global publisher
publisher = None

art = """
  +-+         ____            _             _
  | |        / ___|   _ _ __ (_)_ __   __ _| |_  ___  _ __
  | +--+    | |  | | | | '_ \| | '_ \ / _` | __|/ _ \| '__|
  | |       | |__| |_| | |_) | | | | | (_| | |_| (_) | |
  | |        \____\__,_| .__/|_|_| |_|\__,_|\__|\___/|_|
  | |                  |_|
+-+-+----+
|**    **|  Topics in AI, BGU, Fall 2015
*--*--*--*
*  *  *  *   Maor, Lior, Michael
 **    **
"""


def send_command(command_string):
    print "- sending command %s" % command_string
    publisher.publish(CtrlCommand(None, command_string))


def search():
    send_command("search")


def status():
    send_command("status")


def show_help():
    print "Type a command at the prompt."
    print "Available commands are: search, status, help, quit"


def stop_loop():
    global go
    print "Goodbye!"
    go = False


command_dict = {
    "search": search,
    "status": status,
    "help": show_help,
    "quit": stop_loop
}

if __name__ == '__main__':
    print art
    print

    print "Connecting to ROS...",
    publisher = rospy.Publisher("/cupinator/ctrl/command", CtrlCommand, queue_size=5)
    rospy.init_node("CupinatorCli", anonymous=True)

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


