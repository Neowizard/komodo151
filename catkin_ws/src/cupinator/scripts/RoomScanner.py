#!/usr/bin/env python
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rospy
import tf

import math
import os
import cv2
import cv_bridge

from cupinator.srv import SingleCupRecQuery
from cupinator.msg import AggregationData
from cupinator.msg import RoomScannerCommand


class RoomScanner:
    debug_level = 1
    frame_angle = math.pi / 6
    rotation_target = 0
    rotation_listener = None
    at_target = True

    def __init__(self):
        rospy.Subscriber("cupinator/room_scanner/command", RoomScannerCommand, self.scanner_cb)
        self.rotation_subscriber = rospy.Subscriber("/komodo_1/odom_pub", Odometry, self.rotation_listener_cb)

    def scanner_cb(self, scanner_command):
        """
        Dispatcher of the few RoomScannerCommand types to their respective handlers.
        :param scanner_command: RoomScannerCommand message to dispatch
        :type scanner_command: RoomScannerCommand
        """
        if (scanner_command.command != "start"):
            rospy.loginfo("Command {} not implemented, ignoring message".format(scanner_command.command))
            return None
        if (scanner_command.full):
            self.room_scan()
        else:
            self.stationary_scan()

    def rotation_listener_cb(self, odom):
        if (self.at_target):
            return

        angular_rotation = odom.getTotalAngularTraveled()

        # TODO actually check rotation
        if angular_rotation > self.rotation_target:
            self.at_target = True

    def set_rotation_target(self, rotation_angle):
        initial_odometry = rospy.wait_for_message("/komodo_1/odom_pub", Odometry)
        self.at_target = False
        initial_orientation = (initial_odometry.pose.pose.orientation.x,
                               initial_odometry.pose.pose.orientation.y,
                               initial_odometry.pose.pose.orientation.z,
                               initial_odometry.pose.pose.orientation.w)
        initial_angel = tf.transformations.euler_from_quaternion(initial_orientation)[2]
        self.rotation_target = ((rotation_angle + initial_angel) % 2*math.pi)

    def rotate(self, rotation_angle):
        fname = self.rotate.__name__

        rospy.logdebug("{}: Setting rotation angle to {}".format(fname, rotation_angle))
        self.set_rotation_target(rotation_angle)

        rotation_publisher = rospy.Publisher("/komodo_1/cmd_vel", Twist)
        rotation_twist = Twist()
        rotation_twist.angular.z = 1  # TODO: find optimal angular velocity
        rate = rospy.Rate(10)

        rospy.logdebug("{}: Starting rotation".format(fname))
        while (self.at_target is False):
            rotation_publisher.publish(rotation_twist)
            rate.sleep()

        rotation_twist.angular.z = 0
        rotation_publisher.publish(rotation_twist)

        rospy.logdebug("Arrived at target angle")
        if (self.debug_level >= 1):
            odometry = rospy.wait_for_message("/komodo_1/odom_pub", Odometry)
            rospy.logdebug("{}: Current pose = {}".format(fname, odometry.pose))

    def capture_image(self, file):
        fname = self.capture_image.__name__
        # TODO: implement real camera capture
        DB_path = os.path.join(os.path.dirname(__file__), "MockCamInput/")
        rospy.logdebug("{}: loading image from {}".format(fname, DB_path + file))
        captured_image = cv2.imread(DB_path + file)
        msg_image = cv_bridge.CvBridge().cv2_to_imgmsg(captured_image)
        return msg_image

    def room_scan(self):
        fname = self.room_scan.__name__

        if (self.debug_level >= 1):
            start = time.time()
        else:
            start = 0

        agg_pulisher = rospy.Publisher('/cupinator/cup_recognizer/image_aggregate', AggregationData, queue_size=12)
        self.rotation_target = 0

        # TODO find correct max range
        for frame_idx in xrange(0, (math.pi*2 / self.frame_angle) - 1):
            rospy.logdebug("{}: capturing image #{}".format(fname, frame_idx))
            captured_image = self.capture_image("red_test.jpg")
            aggregation_data = AggregationData()
            aggregation_data.Key = frame_idx
            aggregation_data.Image = captured_image
            agg_pulisher.publish(aggregation_data)

            #mock
            rospy.logdebug("{}: Sleeping for 5 seconds to simulate rotation".format(fname))
            rospy.sleep(5)
            #self.rotate(self.frame_angle)

        get_cupRec_result


    def stationary_scan(self):
        fname = self.stationary_scan.__name__

        if (self.debug_level >= 1):
            start = time.time()
        else:
            start = 0

        rospy.logdebug("{}: Waiting for single_cupRec_query service".format(fname))
        rospy.wait_for_service("/cupinator/cup_recognizer/single_cupRec_query",
                               10)  # Waite up to 10 seconds for service
        try:

            cupRec_srv = rospy.ServiceProxy("cupinator/cup_recognizer/single_cupRec_query", SingleCupRecQuery)
            test_img_file_name = "red_test.jpg"
            message_img = self.capture_image("{0}".format(test_img_file_name))
            rospy.logdebug("{}: Sending {} to single_cupRec".format(fname, test_img_file_name))
            best_match_contour = cupRec_srv(message_img)
            rospy.logdebug("{}: {} cost = {}".format(fname, test_img_file_name, best_match_contour.match_cost))
            '''message_img = self.capture_image("green90.jpg")
            best_match_contour = cupRec_srv(message_img)
            print "green90.jpg cost = ", best_match_contour.match_cost
            message_img = self.capture_image("green180.jpg")
            best_match_contour = cupRec_srv(message_img)
            print "green180.jpg cost = ", best_match_contour.match_cost
            message_img = self.capture_image("green270.jpg")
            best_match_contour = cupRec_srv(message_img)
            print "green270.jpg cost = ", best_match_contour.match_cost
            message_img = self.capture_image("green_scaled.jpg")
            best_match_contour = cupRec_srv(message_img)
            print "green_scaled.jpg cost = ", best_match_contour.match_cost
            message_img = self.capture_image("green_rotated.jpg")
            best_match_contour = cupRec_srv(message_img)
            print "green_rotated.jpg cost = ", best_match_contour.match_cost
            message_img = self.capture_image("green_sprayed.jpg")
            best_match_contour = cupRec_srv(message_img)
            print "green_sprayed.jpg cost = ", best_match_contour.match_cost
            message_img = self.capture_image("green_destroyed.jpg")
            best_match_contour = cupRec_srv(message_img)
            print "green_destroyed.jpg cost = ", best_match_contour.match_cost'''
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed. Exception: %s" % e)

        if (self.debug_level >= 1):
            rospy.logdebug("{}: Total time = {}".format(fname, time.time() - start))


if __name__ == '__main__':
    try:
        rospy.init_node("RoomScanner", anonymous=False, log_level=rospy.DEBUG)
        roomScanner = RoomScanner()
        rospy.spin()

    except rospy.ROSInterruptException, e:
        pass
