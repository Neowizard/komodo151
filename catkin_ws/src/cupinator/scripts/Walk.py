#!/usr/bin/env python
import rospy
import math
import tf
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
minDistance = 0.6
beginPoint = None

def checkCollision(laserData):
    global collisionAlert
    global angleSample
    global minDistance

    #startPoint = (180*512)/(laserData.angle_max - laserData.angle_min)
    #endPoint = laserData.angle_max-startPoint-1
	# We dont have do compute this because angle_max ~ pi/2 and angle_min ~ -pi/2

    #START AT 25 END AT 512-26 (TOTAL OF 512 points) - this will give us 160 degrees
    i=25
    while (i<486):
        if (laserData.ranges[i]<=minDistance and not math.isnan(laserData.ranges[i])):
            collisionAlert = True
            break
        i=i+1


def checkDistanceTraveled(odom):
    global linearDistanceAlert
    global angularDistanceAlert
    global linearDistance
    global angularDistance
    global beginPoint

    #SET VALUE TO BEGIN POINT
    if (beginPoint is None):
        beginPoint = odom.pose
    #


    # COMPUTE LINEAR DISTANCE
    linearTraveled = math.sqrt(math.pow((beginPoint.pose.position.x - odom.pose.pose.position.x),2) + math.pow((beginPoint.pose.position.y - odom.pose.pose.position.y
    ),2))
    #
    if linearTraveled >= linearDistance:
        linearDistanceAlert = True

    #COMPUTE ANGULAR DISTANCE
    Q1 = (beginPoint.pose.orientation.x, beginPoint.pose.orientation.y, beginPoint.pose.orientation.z, beginPoint.pose.orientation.w)
    Q2 = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)

    E1 = tf.transformations.euler_from_quaternion(Q1)
    E2 = tf.transformations.euler_from_quaternion(Q2)

    Y1 = E1[2]
    Y2 = E2[2]

    angularTraveled = Y1-Y2
    #

    if angularTraveled <= -angularDistance:
        angularDistanceAlert = True


def walkUntilCollision(cmd):
    global linearDistance
    global angularDistance
    global collisionAlert
    global linearDistanceAlert
    global angularDistanceAlert
    global PI

    linearDistance = cmd.distance
    angularDistance = cmd.angle

    #rospy.loginfo("")

    walkPub = rospy.Publisher('/komodo_1/cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('/komodo_1/odom_pub', Odometry, checkDistanceTraveled)

    rate = rospy.Rate(10)
    walkTwist = Twist()
    while not (angularDistanceAlert or collisionAlert):
        walkTwist.angular.z = 0.35*PI
        walkPub.publish(walkTwist)
        rate.sleep()

    walkTwist.angular.z = 0
    walkPub.publish(walkTwist)

    while not (linearDistanceAlert or collisionAlert):
        walkTwist.linear.x = 0.5
        walkPub.publish(walkTwist)
        rate.sleep()


    # PUBLISH 0 SPEED TO MAKE IT STOP MOVING
    walkTwist.linear.x = 0.0
    walkPub.publish(walkTwist)
    # ------------------------

    # RESTORE THE VALUES FOR THE GLOBAL VARIABLES
    #collisionAlert = False
    #linearDistanceAlert = False
    #angularDistanceAlert = False
    #linearDistance = 0
    #angularDistance = 0
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
	rospy.spin()
    except rospy.ROSInterruptException:
        pass
