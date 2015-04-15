#!/usr/bin/env python
import time
import rospy
from sensor_msgs.msg import Image
import cv_bridge

from threading import Thread
import Queue
import math
import os
import cv2
import sys

from Walk import Walker
from cupinator.msg import RoomScannerResult
from CupRec import ShapeContextCupRec
from cupinator.msg import RoomScannerCommand


SCAN_FAILED_COST = 1 << 16


class RoomScanner:
    debug_level = 1
    camera_horizontal_arc = 58  # Asus Xiton Pro Live has a 58deg horizontal angle
    rotation_target = 0
    rotation_listener = None
    at_target = True
    cupRec = None
    cupRec_queue = None
    aggregation_thread = None
    abort_scan = True
    walker = None
    bgr_image = None
    depth_image = None
    active_bgr_capture = False
    active_depth_capture = False

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

        self.walker = Walker(False)

        rospy.Subscriber("cupinator/room_scanner/command", RoomScannerCommand, self.scanner_cb)

    def bgr_listener_cb(self, rgb_image):
        """
        """
        fname = "{}::{}".format(self.__class__.__name__, self.bgr_listener_cb.__name__)

        if (not self.active_bgr_capture):
            return
        rospy.logdebug("{}: Storing RGB image".format(fname))
        bridge = cv_bridge.CvBridge()
        try:
            self.bgr_image = bridge.imgmsg_to_cv2(rgb_image, "bgr8")
        except cv_bridge.CvBridgeError, cv_bridge_except:
            rospy.logerr("Failed to convert ROS image message to CvMat\n%s", str(cv_bridge_except))
            return
        self.active_bgr_capture = False

    def depth_listener_cb(self, depth_image):
        """
        """
        fname = "{}::{}".format(self.__class__.__name__, self.depth_listener_cb.__name__)

        if (not self.active_depth_capture):
            return
        rospy.logdebug("{}: Storing depth image".format(fname))

        bridge = cv_bridge.CvBridge()
        try:
            self.depth_image = bridge.imgmsg_to_cv2(depth_image)
        except cv_bridge.CvBridgeError, cv_bridge_except:
            rospy.logerr("Failed to convert ROS image message to CvMat\n%s", str(cv_bridge_except))
            return
        self.active_depth_capture = False

    def scanner_cb(self, scanner_command):
        """
        Dispatcher of the few RoomScannerCommand types to their respective handlers.
        :param scanner_command: RoomScannerCommand message to dispatch
        :type scanner_command: RoomScannerCommand
        """
        fname = "{}::{}".format(self.__class__.__name__, self.scanner_cb.__name__)

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

    def rotate(self, rotation_angle):
        fname = "{}::{}".format(self.__class__.__name__, self.rotate.__name__)

        rospy.logdebug("{}: Rotating {} deg".format(fname, rotation_angle))
        if (rotation_angle < 0):
            rotation_angle += 360

        rotation_angle_rad = ((float)(rotation_angle)/360) * (2 * math.pi)
        rospy.logdebug("{}: Making {}rad angular rotation (using Walker)"
                       .format(fname, rotation_angle_rad))
        self.walker.moveAngular(rotation_angle_rad, True)

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

    def contour_center_in_frame(self, contour, frame):
        """
        Computes the center point of the contour in the given frame
        Uses simple average over the x axis to find the center of the contour.

        :param contour: Contour found in the frame
        :param frame: Reference frame (image)
        :type frame: np.ndarray
        """
        fname = "{}::{}".format(self.__class__.__name__, self.contour_center_in_frame.__name__)

        x_coords_sum = 0
        y_coords_sum = 0
        max_x_coord = frame.shape[1]
        max_y_coord = frame.shape[0]
        rospy.logdebug("{}: Frame shape = {}, max_x_coord = {}, max_y_coord = {}"
                       .format(fname, frame.shape, max_x_coord, max_y_coord))

        for (point_idx, point) in enumerate(contour[:, 0, :]):
            if ((point[0] > max_x_coord) or (point[1] > max_y_coord)):
                rospy.logdebug("{}: Point #{} ({}) in contour is out of bounds of the reference frame (shape = {})"
                               .format(fname, point_idx, point, frame.shape))
            x_coords_sum += point[0]
            y_coords_sum += point[1]

        center_y = y_coords_sum / contour.shape[0]
        center_x = x_coords_sum / contour.shape[0]
        rospy.logdebug("{}: Contour center (x, y) = ({}, {})".format(fname, center_x, center_y))

        return (center_y, center_x)

    def point_angle_in_frame(self, point, frame):
        """
        Gets the horizontal angle of the point in the frame (assumes frame's width is <camera_angle_width>).
        Angle returned is relative to the center of the frame (the center is 0deg)

        :param point: (y, x) coordinates
        :param frame: Reference frame (image)
        :type frame: np.ndarray
        """
        fname = "{}::{}".format(self.__class__.__name__, self.point_angle_in_frame.__name__)

        rospy.logdebug("{}: Frame shape = {}, point = {}".format(fname, frame.shape, point))

        # Frame shape is (Height, Width, Depth), point is (x, y)
        if (((point[1] < 0) or (point[0] < 0)) or
                ((point[1] > frame.shape[1]) or (point[0] > frame.shape[0]))):
            rospy.logdebug("{}: Point {} is out of bounds of the reference frame (shape = {})"
                           .format(fname, point, frame.shape))

        # norm_point_x - x coordinate normalized to [-0.5, 0.5] relative to the center of the frame
        norm_point_x = (float(point[1])/frame.shape[1]) - 0.5

        # The actual angle of the point in the frame (the center of the frame is 0deg)
        horizontal_angle = norm_point_x * self.camera_horizontal_arc
        rospy.logdebug("{}: norm_point_x = {}, horizontal_angle = {}".format(fname, norm_point_x, horizontal_angle))

        return horizontal_angle

    def capture_image(self, mock_file=None):
        fname = "{}::{}".format(self.__class__.__name__, self.capture_image.__name__)
        
        rospy.logdebug("{}: Attempting to capture BGR and depth image".format(fname))
        '''rgb_image = rospy.wait_for_message("/komodo_1/komodo_1_Asus_Camera/rgb/image_raw", Image, timeout=10)
        depth_image = rospy.wait_for_message("/komodo_1/komodo_1_Asus_Camera/depth/image_raw", Image, timeout=10)
        bridge = cv_bridge.CvBridge()
        try:
            cv_bgr_image = bridge.imgmsg_to_cv2(rgb_image, "bgr8")
            #cv_bgr_image = cv2.cvtColor(cv_bgr_image, cv2.COLOR_RGB2BGR)
            #cv_depth_image = CvBridge().imgmsg_to_cv2(depth_image)
        except cv_bridge.CvBridgeError, cv_bridge_except:
            rospy.logerr("Failed to convert ROS image message to CvMat\n%s", str(cv_bridge_except))
            return
        '''

        # Capture RGB image
        rospy.logdebug("{}: Setting up bgr subscriber".format(fname))
        rgb_listener = rospy.Subscriber("/komodo_1/komodo_1_Asus_Camera/rgb/image_raw",
                                        Image, self.bgr_listener_cb)
        rospy.sleep(1)  # We sleep here since the Asus Xiton driver requires a short calibration time whenever
                        # it starts publishing images (it actually publishes only when there's a subscriber registered)
        self.active_bgr_capture = True
        while (self.active_bgr_capture):
            pass  # Busy wait for an image to be captured. WARNING: No timeout - Deadlock potential
        rospy.logdebug("{}: BGR image acquired".format(fname))
        rgb_listener.unregister()

        # Capture depth image
        rospy.logdebug("{}: Setting up depth subscriber".format(fname))
        depth_listener = rospy.Subscriber("/komodo_1/komodo_1_Asus_Camera/depth/image_raw",
                                          Image, self.depth_listener_cb)
        rospy.sleep(1)  # As in the BGR listener, we need to sleep here to allow the Xiton driver to calibrate
        self.active_depth_capture = True
        rospy.logdebug("{}: Triggering depth capture".format(fname))
        while (self.active_depth_capture):
            pass  # Busy wait for an image to be captured. WARNING: No timeout - Deadlock potential
        rospy.logdebug("{}: Depth image acquired".format(fname))
        depth_listener.unregister()

        if (self.debug_level >= 2):
            cv2.imshow("rgb", self.bgr_image)
            cv2.imshow("depth", self.depth_image)
            cv2.waitKey()

        rospy.logdebug("{}: Captured BGR and depth images (shape = {})".format(fname, self.bgr_image.shape))
        # mock
        #mock_path = os.path.join(os.path.dirname(__file__), "MockCamInput/")
        #rospy.logdebug("{}: loading image from {}".format(fname, mock_path + mock_file))
        #captured_image = cv2.imread(mock_path + mock_file)
        return self.bgr_image, self.depth_image

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
        #inputs = ["green_destroyed.jpg", "red_test.jpg", "IMG_scaled.jpg", "green.jpg", "green180.jpg",
        #          "green_scaled.jpg", "green270.jpg"]
        # end mock

        #rospy.loginfo("{}: Setting camera arg to 180deg for testing".format(fname))
        #self.camera_horizontal_arc = 180

        # 360deg scan
        frames_per_circle = (int)(360.0 / self.camera_horizontal_arc)
        frames = []
        for frame_idx in xrange(0, 6):#frames_per_circle):
            # Handle aborts. Flush queue
            if (self.abort_scan):
                rospy.loginfo("{}: Clearing queue and aborting scan".format(fname))
                while (not self.cupRec_queue.empty()):
                    self.cupRec_queue.get(block=False)
                rospy.loginfo("{}: Cleared quforeue. Worker thread will halt after current job")
                return
            # Capture image
            rospy.loginfo("{}: capturing image #{}".format(fname, frame_idx))
            cap_rgb_img, cap_depth_img = self.capture_image()
            frames.insert(frame_idx, (cap_rgb_img, cap_depth_img))

            # Send image to worker thread
            self.cupRec_queue.put((cap_rgb_img, frame_idx))

            # Rotate to next frame
            #mock
            #rospy.logdebug("{}: Sleeping for 5 seconds to simulate rotation".format(fname))
            #rospy.sleep(5)
            #end mock
            rospy.logdebug("{}: Rotating to get new frame ({}deg)"
                           .format(fname, self.camera_horizontal_arc))
            self.rotate(self.camera_horizontal_arc)
            if (self.debug_level >= 1):
                rospy.logdebug("{}: Sleeping for debugability".format(fname))
                rospy.sleep(5)

        # Wait for worker thread
        rospy.logdebug("{}: Waiting for aggregator thread to finish processing images".format(fname))
        self.cupRec_queue.join()

        # Get result from CupRec
        best_match = self.cupRec.cupRec_aggregate_query(True)
        if (best_match is None):
            match_frame_idx = -1
            matched_contour = None
            match_cost = SCAN_FAILED_COST
        else:
            match_frame_idx = best_match[2]
            matched_contour = best_match[1]
            match_cost = best_match[0]

        self.publish_scan_result(match_cost, matched_contour, frames[match_frame_idx], match_frame_idx)

        if (self.debug_level >= 1):
            rospy.logdebug("{}: Total time = {}".format(fname, time.time() - start))

    def publish_scan_result(self, match_cost, matched_contour, match_frame, match_frame_idx=0):
        """

        :param match_cost: Cost of best match. SCAN_FAILED_COST (= 2^16) indicates no cup was found (scan failed)
        :param matched_contour: Contour in match frame that best matches the DB cup (None on scan failed)
        :param match_frame_idx: Frame in which cup was found (defaults to 0 for stationary scan). -1 on scan failed
        :param match_frame: (bgr, depth) cv-images taken at <match_frame_idx>
        """
        fname = "{}::{}".format(self.__class__.__name__, self.publish_scan_result.__name__)
        scan_result = RoomScannerResult()

        if (match_cost >= SCAN_FAILED_COST):
            match_angle = 0
            match_distance = -1
        else:
            # Calculate match angle and distance
            rospy.logdebug("{}: Computing angle in frame from contour".format(fname))
            match_center_point = self.contour_center_in_frame(matched_contour, match_frame[0])
            match_frame_angle = self.point_angle_in_frame(match_center_point, match_frame[0])
            match_angle = self.camera_horizontal_arc * match_frame_idx + match_frame_angle
            # Depth images measure distance in mm, the walker expects cm (so div by 10)
            match_distance = match_frame[1].item(match_center_point) / 10
            rospy.loginfo("{}: best match cost = {} at frame {}. Angle (in frame) = {}deg ({}deg), Distance = {}cm"
                          .format(fname, match_cost, match_frame_idx, match_angle, match_frame_angle, match_distance))

        scan_result.cost = match_cost
        scan_result.angle = match_angle
        scan_result.distance = match_distance

        # Publish message
        rospy.logdebug("{}: Publishing scan result (cost = {}, angle = {}deg, distance = {}cm)"
                       .format(fname, scan_result.cost, scan_result.angle, scan_result.distance))
        scan_publisher = rospy.Publisher("/cupinator/room_scanner/result", RoomScannerResult, queue_size=2)
        scan_publisher.publish(scan_result)

    def stationary_scan(self):
        fname = "{}::{}".format(self.__class__.__name__, self.stationary_scan.__name__)

        start = time.time()

        # Capture image
        cap_rgb_img, cap_depth_img = self.capture_image()

        #mock
        #test_img_file_name = "red_test.jpg"
        # rospy.logdebug("{}: Sending {} to cupRec".format(fname, test_img_file_name))
        #end mock

        # Find best match contour
        best_match = self.cupRec.cupRec_single_query(cap_rgb_img)

        if (best_match is None):
            matched_contour = None
            match_cost = SCAN_FAILED_COST
        else:
            matched_contour = best_match[1]
            match_cost = best_match[0]

        self.publish_scan_result(match_cost, matched_contour, (cap_rgb_img, cap_depth_img))

        '''
        # Calculate match angle and distance
        rospy.logdebug("{}: Computing angle in frame from contour".format(fname))
        match_center_point = self.contour_center_in_frame(matched_contour, cap_rgb_img)
        match_angle_in_frame = self.point_angle_in_frame(match_center_point, cap_rgb_img)
        match_distance = frames[match_frame][1].item(match_center_point)

        # Create result to publish
        rospy.logdebug("{}: Constructing RoomScannerResult message".format(fname))
        scan_result = RoomScannerResult()
        scan_result.cost = match_cost
        scan_result.angle = match_angle_in_frame
        scan_result.distance = match_distance

        rospy.logdebug("{}: Publishing scan result (cost = {}, angle = {}, distance = {})"
                       .format(fname, scan_result.cost, scan_result.angle, scan_result.distance))
        scan_publisher = rospy.Publisher("/cupinator/room_scanner/result", RoomScannerResult, queue_size=2)
        scan_publisher.publish(scan_result)
        '''

        if (self.debug_level >= 1):
            rospy.logdebug("{}: Total time = {}".format(fname, time.time() - start))


if __name__ == '__main__':
    try:
        roomScanner = RoomScanner()
        rospy.spin()

    except rospy.ROSInterruptException, e:
        pass
