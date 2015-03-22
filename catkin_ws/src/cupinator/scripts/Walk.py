#!/usr/bin/env python
import rospy
import math
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cupinator.msg import WalkCommand
from cupinator.msg import WalkResult

class Walker:
    collisionAlert = False
    linearDistanceAlert = False
    angularDistanceAlert = False
    linearDistance = 0
    angularDistance = 0
    minDistance = 0.6
    beginPoint = None
    odomCheck = False

    def __init__(self, accept_command=True):
        rospy.Subscriber('/komodo_1/scan', LaserScan, self.checkCollision)
        if (accept_command):
            rospy.Subscriber('/cupinator/walk/command', WalkCommand, self.walkUntilCollision)
        rospy.Subscriber('/komodo_1/odom_pub', Odometry, self.checkDistanceTraveled)


    def checkCollision(self, laserData):
        '''global self.collisionAlert
        global self.minDistance'''

        #startPoint = (180*512)/(laserData.angle_max - laserData.angle_min)
        #endPoint = laserData.angle_max-startPoint-1
        # We dont have do compute this because angle_max ~ pi/2 and angle_min ~ -pi/2

        #START AT 25 END AT 512-26 (TOTAL OF 512 points) - this will give us 160 degrees
        i=25
        while (i<486):
            if (laserData.ranges[i]<=self.minDistance and not math.isnan(laserData.ranges[i])):
                self.collisionAlert = True
                break
            i=i+1


    def checkDistanceTraveled(self, odom):
        '''global self.linearDistanceAlert
        global self.angularDistanceAlert
        global linearDistance
        global angularDistance
        global self.beginPoint
        global self.odomCheck'''


        if (self.odomCheck):
            #SET VALUE TO BEGIN POINT
            if (self.beginPoint is None):
                self.beginPoint = odom.pose
            #


            # COMPUTE LINEAR DISTANCE
            linearTraveled = math.sqrt(math.pow((self.beginPoint.pose.position.x - odom.pose.pose.position.x),2) + math.pow((self.beginPoint.pose.position.y - odom.pose.pose.position.y
            ),2))
            #
            if linearTraveled >= self.linearDistance:
                self.linearDistanceAlert = True

            # COMPUTE ANGULAR DISTANCE
            Q1 = (self.beginPoint.pose.orientation.x, self.beginPoint.pose.orientation.y, self.beginPoint.pose.orientation.z, self.beginPoint.pose.orientation.w)
            Q2 = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)

            E1 = tf.transformations.euler_from_quaternion(Q1)
            E2 = tf.transformations.euler_from_quaternion(Q2)

            # TRANSFORM RANGE FROM [0,PI,-PI,0] TO [0,2PI]
            Y1 = 2*math.pi + E1[2] if E1[2] < 0 else E1[2]
            Y2 = 2*math.pi + E2[2] if E2[2] < 0 else E2[2]
            #

            # CHECK IF THE WHEELS PASSED THE RESET POINT IN THE CIRCLE
            Y2 = 2*math.pi+Y2 if Y1>Y2 else Y2
            #

            angularTraveled = Y2 - Y1
            #

            if angularTraveled >= self.angularDistance:
                self.angularDistanceAlert = True

    def moveAngular(self, distance):
        '''global self.odomCheck
        global angularDistance
        global self.angularDistanceAlert
        global self.collisionAlert
        global self.beginPoint'''

        self.angularDistanceAlert = False
        self.angularDistance = distance

        walkPub = rospy.Publisher('/komodo_1/cmd_vel', Twist, queue_size=10)

        self.beginPoint = None
        self.odomCheck = True

        rate = rospy.Rate(10)
        walkTwist = Twist()
        while not (self.angularDistanceAlert or self.collisionAlert):
            walkTwist.angular.z = 0.3*math.pi
            walkPub.publish(walkTwist)
            rate.sleep()

        # PUBLISH 0 SPEED TO MAKE IT STOP MOVING
        walkTwist.angular.z = 0
        walkPub.publish(walkTwist)
        #

        self.odomCheck = False

    def moveLinear(self, distance):
        '''global self.odomCheck
        global linearDistance
        global self.linearDistanceAlert
        global self.collisionAlert
        global self.beginPoint'''

        self.linearDistanceAlert = False
        self.linearDistance = distance

        walkPub = rospy.Publisher('/komodo_1/cmd_vel', Twist, queue_size=10)

        self.beginPoint = None
        self.odomCheck = True

        rate = rospy.Rate(10)
        walkTwist = Twist()
        while not (self.linearDistanceAlert or self.collisionAlert):
            walkTwist.linear.x = 0.5
            walkPub.publish(walkTwist)
            rate.sleep()


        # PUBLISH 0 SPEED TO MAKE IT STOP MOVING
        walkTwist.linear.x = 0.0
        walkPub.publish(walkTwist)
        #

        self.odomCheck = False


    def walkUntilCollision(self, cmd):

        self.moveAngular((cmd.angle*math.pi)/180)

        self.moveLinear(cmd.distance)

        # PUBLISH WALK RESULT
        walkPub = rospy.Publisher('/cupinator/walk/result', WalkResult, queue_size=2)
        res = WalkResult()
        res.result = not self.collisionAlert # if self.collisionAlert is true, we failed to move all the way
        walkPub.publish(res)
        # ------------------

if __name__ == '__main__':
    try:
        rospy.init_node('Walk', anonymous=True)
        walker = Walker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
