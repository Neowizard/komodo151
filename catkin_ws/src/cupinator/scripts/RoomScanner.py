#!/usr/bin/env python
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rospy
import tf

from threading import Thread
import Queue
import math
import os
import cv2

from cupinator.msg import RoomScannerResult
from CupRec import ShapeContextCupRec
from cupinator.msg import RoomScannerCommand


class RoomScanner:
    debug_level = 1
    camera_horizontal_width = 58  # Asus Xiton Pro Live has a 58deg horizontal angle
    rotation_target = 0
    rotation_listener = None
    at_target = True
    cupRec = None
    cupRec_queue = None
    aggregation_thread = None
    abort_scan = True

    def __init__(self):
        if (self.debug_level >= 1):
            rospy_debug_level = rospy.DEBUG
        else:
            rospy_debug_level = None
        rospy.init_node("RoomScanner", anonymous=False, log_level=rospy_debug_level)

        self.cupRec = ShapeContextCupRec()
        self.cupRec_queue = Queue.Queue()
        self.aggregation_thread = Thread(target=self.aggregation_thread_cb)
        self.aggregation_thread.setDaemon(True)
        self.aggregation_thread.start()

        rospy.Subscriber("cupinator/room_scanner/command", RoomScannerCommand, self.scanner_cb)
        self.rotation_subscriber = rospy.Subscriber("/komodo_1/odom_pub", Odometry, self.rotation_listener_cb)

    def scanner_cb(self, scanner_command):
        fname = "{}::{}".format(self.__class__.__name__, self.scanner_cb.__name__)
        """
        Dispatcher of the few RoomScannerCommand types to their respective handlers.
        :param scanner_command: RoomScannerCommand message to dispatch
        :type scanner_command: RoomScannerCommand
        """

        rospy.loginfo("{}: Processing RoomScanner command {} (full = {})".format(fname, scanner_command.command,
                                                                                 scanner_command.full))

        if (scanner_command.command == "stop"):
            self.abort_scan = True
        elif (scanner_command.command == "start"):
            self.abort_scan = False
            if (scanner_command.full):
                scanning_thread = Thread(target=self.room_scan)
                scanning_thread.setDaemon(False)
                scanning_thread.start()
            else:
                self.stationary_scan()
        else:
            rospy.loginfo("Command {} not implemented, ignoring message".format(scanner_command.command))
            return None

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
        self.rotation_target = ((rotation_angle + initial_angel) % 2 * math.pi)

    def rotate(self, rotation_angle):
        fname = "{}::{}".format(self.__class__.__name__, self.rotate.__name__)

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

    def aggregation_thread_cb(self):
        """
        Worker background thread. Constantly tries to dequeue the producer-consumer queue for (snapshots, key)
        tuples. Each tuple is sent to a CupRec for processing and aggregation.

        :return: None
        """
        fname = "{}::{}".format(self.__class__.__name__, self.aggregation_thread_cb.__name__)
        rospy.logdebug("{}: Aggregation thread ready".format(fname))
        while (True):
            aggregation_data = self.cupRec_queue.get()
            rospy.logdebug("{}: Sending aggregation data with key = {} to CupRec".format(fname, aggregation_data[1]))
            self.cupRec.aggregate_image(aggregation_data)
            self.cupRec_queue.task_done()

    def contour_angle_in_frame(self, contour, frame):
        """
        Gets the horizontal angle of the contour in the frame (assumes frame's width is <camera_angle_width>).
        Uses simple average over the x axis to find the center of the contour.
        Angle returned is relative to the center of the frame (the center is 0deg)

        :param contour: Contour found in the frame
        :param frame: Reference frame (image)
        :type frame: np.ndarray
        """
        fname = "{}::{}".format(self.__class__.__name__, self.contour_angle_in_frame.__name__)

        x_coords_sum = 0
        max_x_coord = frame.shape[2]

        for (point_idx, point) in enumerate(contour[:, 0, :]):
            if (point[0] > max_x_coord):
                rospy.logdebug("{}: Point #{} in contour is out of bounds of the reference frame (shape = {})"
                               .format(fname, point_idx, frame.shape))
            x_coords_sum += point[0]

        # relative_center_x is the position of the center of the contour relative to the center of the frame
        # normalized to [0, 1]
        relative_center_x = x_coords_sum / contour.shape[0] - 0.5

        # The actual angle of the center of the contour in the frame (the center of the frame is 0deg)
        center_point_angle = ((float)(relative_center_x)/frame.shape[2]) * self.camera_horizontal_width

        return center_point_angle


    def capture_image(self, mock_file):
        fname = "{}::{}".format(self.__class__.__name__, self.capture_image.__name__)
        # TODO: implement real camera capture
        mock_path = os.path.join(os.path.dirname(__file__), "MockCamInput/")
        rospy.logdebug("{}: loading image from {}".format(fname, mock_path + mock_file))
        captured_image = cv2.imread(mock_path + mock_file)
        return captured_image

    def room_scan(self):
        """
        Performs a 360deg scan taking snapshots at every <self.camera_horizontal_angle> and posting it for a worker
        thread. Each snapshot is searched for a cup. When the best match for a cup is found, the angle of the contour
        in the snapshot is calculated and the contour with its angle is published under the topic
        /cupinator/room_scanner/result

        If <self.abort_scan> is set, the function clears the worker thread's queue and aborts

        :return: None
        """
        fname = "{}::{}".format(self.__class__.__name__, self.room_scan.__name__)

        start = time.time()

        # mock
        inputs = ["green_destroyed.jpg", "red_test.jpg", "IMG_scaled.jpg", "green.jpg", "green180.jpg",
                  "green_scaled.jpg", "green270.jpg"]
        # end mock

        # 360deg scan
        frames_per_circle = (int)(math.ceil(360.0 / self.camera_horizontal_width))
        frames = []
        for frame_idx in xrange(0, frames_per_circle):
            # Handle aborts. Flush queue
            if (self.abort_scan):
                rospy.loginfo("{}: Clearing queue and aborting scan".format(fname))
                while (not self.cupRec_queue.empty()):
                    self.cupRec_queue.get(block=False)
                rospy.loginfo("{}: Cleared queue. Worker thread will halt after current job")
                return
            # Capture image
            rospy.loginfo("{}: capturing image #{}".format(fname, frame_idx))
            cap_img = self.capture_image(inputs[frame_idx])
            frames.insert(cap_img, frame_idx)

            # Send image to worker thread
            self.cupRec_queue.put((cap_img, frame_idx))

            # Rotate to next frame
            #mock
            rospy.logdebug("{}: Sleeping for 5 seconds to simulate rotation".format(fname))
            rospy.sleep(5)
            #self.rotate(self.frame_angle)

        # Wait for worker thread
        rospy.logdebug("{}: Waiting for aggregator thread to finish processing images".format(fname))
        self.cupRec_queue.join()

        # Get result from CupRec
        best_match = self.cupRec.cupRec_aggregate_query(True)
        match_frame = best_match[2]
        matched_contour = best_match[1]
        match_cost = best_match[0]

        # Calculate match angle
        rospy.logdebug("{}: Computing angle in frame from contour".format(fname))
        match_angle_in_frame = self.contour_angle_in_frame(matched_contour, frames[match_frame])
        absolute_match_angle = frames_per_circle * match_frame + match_angle_in_frame
        rospy.loginfo("{}: best match cost = {} at frame {}. Frame angle (absolute) = {}deg ({}deg)"
                      .format(fname, match_cost, match_frame, match_angle_in_frame, absolute_match_angle))

        # Construct result message
        rospy.logdebug("{}: Constructing RoomScannerResult message".format(fname))
        scan_result = RoomScannerResult()
        scan_result.cost = match_cost
        scan_result.angle = absolute_match_angle

        # Publish message
        rospy.logdebug("{}: Publishing scan result (cost = {}, angle = {}"
                       .format(fname, scan_result.cost, scan_result.angle))
        scan_publisher = rospy.Publisher("/cupinator/room_scanner/result", RoomScannerResult, queue_size=2)
        scan_publisher.publish(scan_result)

        if (self.debug_level >= 1):
            rospy.logdebug("{}: Total time = {}".format(fname, time.time() - start))

    def stationary_scan(self):
        fname = "{}::{}".format(self.__class__.__name__, self.stationary_scan.__name__)

        start = time.time()

        # Capture image
        test_img_file_name = "red_test.jpg"
        cap_img = self.capture_image("{0}".format(test_img_file_name))

        # Find best match contour
        rospy.logdebug("{}: Sending {} to cupRec".format(fname, test_img_file_name))
        best_match = self.cupRec.cupRec_single_query(cap_img)
        matched_contour = best_match[1]
        match_cost = best_match[0]

        # Calculate match angle
        rospy.logdebug("{}: Computing angle in frame from contour".format(fname))
        match_angle_in_frame = self.contour_angle_in_frame(matched_contour, cap_img)

        # Create result to publish
        rospy.logdebug("{}: Constructing RoomScannerResult message".format(fname))
        scan_result = RoomScannerResult()
        scan_result.cost = match_cost
        scan_result.angle = match_angle_in_frame

        rospy.logdebug("{}: Publishing scan result (cost = {}, angle = {})"
                       .format(fname, scan_result.cost, scan_result.angle))
        scan_publisher = rospy.Publisher("/cupinator/room_scanner/result", RoomScannerResult, queue_size=2)
        scan_publisher.publish(scan_result)


        if (self.debug_level >= 1):
            rospy.logdebug("{}: Total time = {}".format(fname, time.time() - start))


if __name__ == '__main__':
    try:
        roomScanner = RoomScanner()
        rospy.spin()

    except rospy.ROSInterruptException, e:
        pass
