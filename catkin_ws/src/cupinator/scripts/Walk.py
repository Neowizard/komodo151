#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


collisionAlert = False

def checkCollision(laserData):
    global collisionAlert


    if not collisionAlert:


    #print("[checkProxCB] Range: " + str(rangeData.range) + " proxAlert: " + str(collisionAlert))


def scanChecker():
    rospy.Subscriber('/komodo_1/scan', LaserScan, checkCollision)
    

def walkUntilCollision():
    walkPub = rospy.Publisher('/komodo_1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('Walk', anonymous=True)
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
	#print("[Publisher] proxAlert: " + str(collisionAlert))
        walkTwist = Twist()
        if (collisionAlert == False):
            walkTwist.linear.x = 0.7
        else:
            walkTwist.linear.x = 0.0

        walkPub.publish(walkTwist)
        rate.sleep()

def waitForCommand():
    rospy.Subscriber('/cupinator/walk/command',

if __name__ == '__main__':
    try:
        waitForCommand()
        scanChecker()
        walkUntilCollision()
    except rospy.ROSInterruptException:
        pass
