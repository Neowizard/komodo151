#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cupinator.msg import WalkCommand
from cupinator.msg import WalkResult

collisionAlert = False
linearDistanceAlert = False
angularDistanceAlert = False
linearDistance = 0
angularDistance = 0
PI = 3.14
angleSample = 180
minDistance = 0.25

def checkCollision(laserData):
    global collisionAlert
    global angleSample
    global minDistance

    startPoint = (180*512)/(laserData.angle_max - laserData.angle_min)
    endPoint = laserData.angle_max-startPoint-1

    for i in range(startPoint,endPoint):
        if laserData.ranges[i]<=minDistance:
            collisionAlert = True
            break


def checkDistanceTraveled(odom):
    global linearDistanceAlert
    global angularDistanceAlert
    global linearDistance
    global angularDistance
    # TODO: TRAVELED
    linearTraveled = odom.getTotalLinearTraveled()
    if linearTraveled >= linearDistance:
        linearDistanceAlert = True

    angularTraveled = odom.getTotalAngularTraveled()
    if angularTraveled > angularDistance:
        angularDistanceAlert = True



def walkUntilCollision(cmd):
    global linearDistance
    global angularDistance
    global collisionAlert
    global linearDistanceAlert
    global angularDistanceAlert
    global PI

    linearDistance = cmd.distance

    walkPub = rospy.Publisher('/komodo_1/cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('/komodo_1/odom_pub', Odometry, checkDistanceTraveled)

    rate = rospy.Rate(10)
    walkTwist = Twist()
    while not (angularDistanceAlert or collisionAlert):
        walkTwist.angular.z = 0.3*PI
        walkPub.publish(walkTwist)
        rate.sleep()

    walkTwist.angular.z = 0
    walkPub.publish(walkTwist)

    while not (linearDistanceAlert or collisionAlert):
        walkTwist.linear = 0.5
        walkPub.publish(walkTwist)
        rate.sleep()


    # PUBLISH 0 SPEED TO MAKE IT STOP MOVING
    walkTwist.linear.x = 0.0
    walkPub.publish(walkTwist)
    # ------------------------

    # RESTORE THE VALUES FOR THE GLOBAL VARIABLES
    collisionAlert = False
    linearDistanceAlert = False
    angularDistanceAlert = False
    linearDistance = 0
    angularDistance = 0
    # -----------------

    # PUBLISH WALK RESULT
    walkPub = rospy.Publisher('/cupinator/walk/result', WalkResult, queue_size=2)
    res = WalkResult()
    res.result = not collisionAlert # if collisionAlert is true, we failed to move all the way
    walkPub.publish(res)
    # ------------------

if __name__ == '__main__':
    try:
        rospy.init_node('Walk', anonymous=True)
        rospy.Subscriber('/komodo_1/scan', LaserScan, checkCollision)
        rospy.Subscriber('/cupinator/walk/command', WalkCommand, walkUntilCollision)
    except rospy.ROSInterruptException:
        pass
