#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


proxAlert = False

def checkProxCB(rangeData):
    global proxAlert
    proxAlert = (rangeData.range < 1)
    print("[checkProxCB] Range: " + str(rangeData.range) + " proxAlert: " + str(proxAlert) )


def proxChecker():
    rospy.Subscriber('/komodo_1/Rangers/Rear_URF', Range, checkProxCB)
    

def Publisher():
    walkPub = rospy.Publisher('/komodo_1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('walkProx', anonymous=True)
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
	print("[Publisher] proxAlert: " + str(proxAlert))	
        walkTwist = Twist()
        if (proxAlert == False):
            walkTwist.linear.x = -0.6
        else:
            walkTwist.linear.x = 0.0
        walkPub.publish(walkTwist)
        rate.sleep()

if __name__ == '__main__':
    try:
	proxChecker()
        Publisher()
    except rospy.ROSInterruptException:
        pass
