#!/usr/bin/env python
import heapq
import sys
from std_msgs.msg import Int16
from cupinator.srv import CupRecQueryRequest
from cupinator.srv import CupRecQueryResponse
from cupinator.srv import CupRecQuery
from cupinator.srv import SingleCupRecQueryRequest
from cupinator.srv import SingleCupRecQueryResponse
from cupinator.srv import SingleCupRecQuery
import rospy
import time
import numpy as np
import cv2
import os
from threading import Lock
import cv_bridge
from cupinator.msg import AggregationData
import cupRec.SC


class ShapeContextCupRec:
    """
    main_channel: The main channel to filter the image with
    main_lower_th: Low bound before filtering out the main channel
                                (higher means more aggressive filtering)
    secondary_factor: Factor for filtering out the secondary channels
                                (higher means more aggressive filtering)
    sample_size: Number of contour samples to send to SC algorithm
    """

    lower_red = np.array([0, 80, 50])
    upper_red = np.array([4, 255, 255])
    lower_high_red = np.array([172, 80, 50])
    upper_high_red = np.array([179, 255, 255])

    lower_green = np.array([45, 40, 40])
    upper_green = np.array([75, 255, 255])

    sample_size = 0
    debug_level = 3

    lock = Lock()
    shapecontext_DB = []
    contour_heap = []

    def __init__(self, sample_size=100):
        self.sample_size = sample_size
        self.load_shapecontext_DB()

    def load_shapecontext_DB(self):
        fname = self.load_shapecontext_DB.__name__
        rospy.loginfo("Loading DB")
        start_time = time.time()

        # TODO: Store computed SC on disk to reduce startup time
        DB_path = os.path.join(os.path.dirname(__file__), "cupRec/DB/")
        for DB_file in os.listdir(DB_path):
            if DB_file.endswith(".jpg") | DB_file.endswith(".png") | DB_file.endswith(".bmp"):
                rospy.loginfo("{}: Computing SC for {}".format(fname, DB_file))
                DB_img = cv2.imread(DB_path + DB_file)
                shapecontexts = self.compute_shapecontext(DB_img)

                rospy.logdebug("{}: {} contours for DB image {}".format(fname, len(shapecontexts), DB_file))
                if (self.debug_level >= 3):
                    for (idx, (img_contour, contour_SC)) in enumerate(shapecontexts):
                        contour_img = np.zeros((DB_img.shape[0], DB_img.shape[1]))
                        cv2.drawContours(contour_img, img_contour, -1, 255)
                        cv2.imshow("contour {0}_{1}".format(DB_file, idx), contour_img)
                    cv2.waitKey(0)

                for (img_contour, contour_SC) in shapecontexts:
                    self.shapecontext_DB.append((img_contour, contour_SC))

        if (self.debug_level >= 1):
            end_time = time.time()
            rospy.logdebug("{}: cupinator/cupRec: DB load time = {}".format(fname, end_time - start_time))

    def sample_contours_from_binary_img(self, bw_img, sample_size, low_th=50, high_th=200):
        """
        :param bw_img: (ndarray) Image to get contour sample from
        :param sample_size: (int) Maximal number of samples
        :return: (array) A 2D array of coordinates (x,y) sampled from the contour
        """
        erosion_kernel = np.asarray([[1, 1, 1], [1, 1, 1], [1, 1, 1]], np.uint8)
        bw_img = cv2.dilate(bw_img, erosion_kernel)
        contour_img = cv2.Canny(bw_img, low_th, high_th)

        if (self.debug_level >= 2):
            cv2.imshow("canny_dialated", contour_img)

        contours, dont_care = cv2.findContours(contour_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        contours_samples = []

        for contour in contours:
            contour_samples_count = sample_size
            contour_samples = np.ndarray((contour_samples_count, 1, 2), contours[0].dtype)

            marker_step = max((float)(len(contour)) / (float)(sample_size), 1)
            contour_marker = 0
            contour_idx = 0
            for sample_idx in xrange(0, contour_samples_count - 1):
                if (contour_idx >= len(contour)):
                    break
                contour_samples[sample_idx, :, :] = contour[contour_idx].copy()

                contour_marker += marker_step
                contour_idx = round(contour_marker)
            if (contour_idx < len(contour)):
                contours_samples.append(contour_samples)

        return contours_samples

    def compute_shapecontext(self, cv_img):

        """
        :rtype : List of (Contour, ShapeContext) elements
        :param cv_img: Input BGR image
        """

        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        binary_img = cv2.inRange(hsv_img, self.lower_red, self.upper_red)
        binary_img += cv2.inRange(hsv_img, self.lower_high_red, self.upper_high_red)

        if (self.debug_level >= 2):
            cv2.imshow("binary_img", binary_img)
            cv2.waitKey()

        input_samples = self.sample_contours_from_binary_img(binary_img, self.sample_size)

        sc = cupRec.SC.SC()

        contours_SC = []
        for input_contour_samples in input_samples:
            contour_SC = sc.compute(input_contour_samples[:, 0, :])
            contours_SC.append((input_contour_samples, contour_SC))

        return contours_SC

    def cupRec_single_query_cb(self, single_snapshot_query):
        fname = self.cupRec_single_query_cb.__name__

        """

        :param single_snapshot_query:
        :type single_snapshot_query: SingleCupRecQueryRequest
        """

        rospy.logdebug("{}: Starting single query cup recognition...".format(fname))

        if (self.debug_level >= 1):
            start = time.time()
        else:
            start = 0

        try:
            rospy.logdebug("{}: Converting ROS img format to CvMat".format(fname))
            cv_img = cv_bridge.CvBridge().imgmsg_to_cv2(single_snapshot_query.snapshot)
        except cv_bridge.CvBridgeError, cv_bridge_except:
            rospy.logerr("{}: Failed to convert ROS image message to CvMat\n{}".format(fname, cv_bridge_except))
            return None

        rospy.logdebug("{}: Computing shape contexts for contours in image".format(fname))
        contour_match_list = self.compute_contours_shapecontext_cost(cv_img)

        min_cost = sys.maxint
        min_cost_idx = -1
        for (match_idx, (match_cost, contour, DB_contour)) in enumerate(contour_match_list):
            if (match_cost < min_cost):
                min_cost = match_cost
                min_cost_idx = match_idx
                rospy.logdebug("{}: Setting best-cost contour to contour_{}".format(fname, match_idx))

        rospy.logdebug("{}: Creating query response".format(fname))
        response = SingleCupRecQueryResponse()
        response_img = np.zeros((cv_img.shape[0], cv_img.shape[1], 1))
        cv2.drawContours(response_img, [contour_match_list[min_cost_idx][1]], 0, 255)

        try:
            rospy.logdebug("{}: Converting CvMat to ROS img".format(fname))
            response.match_contour = cv_bridge.CvBridge().cv2_to_imgmsg(response_img)
        except cv_bridge.CvBridgeError, cv_bridge_except:
            rospy.logerr(
                "{}: Failed to convert contour image from CvMat to Ros Msg\n{}".format(fname, cv_bridge_except))
            return
        response.match_cost = contour_match_list[min_cost_idx][0]

        if (self.debug_level >= 1):
            end = time.time()
            rospy.logdebug("{}: Query time = {}".format(fname, end - start))
        rospy.logdebug("{}: Sending response".format(fname))

        return response

    def compute_contours_shapecontext_cost(self, cv_img):
        fname = self.compute_contours_shapecontext_cost.__name__
        """
        Computes the shapecontext of all the contours in cv_img, and stores them in a list
        along with the cost of matching each contour to its best match in the DB
        :param cv_img: input OpenCV2 image
        :type cv_img: numpy.ndarray
        :return: list of (match cost, contour, matched DB contour)
                        contours are ndarray[self.sample_size, 1, 2]
        """

        if (self.debug_level >= 2):
            cv2.imshow('img', cv_img)
            cv2.waitKey(0)
        # end debug
        ''' Compute the shapecontext for all the contours in the image.
                    Note: Ignores contours nested inside other contours '''
        rospy.logdebug("Computing shape context for image")
        contours_SC = self.compute_shapecontext(cv_img)
        if (self.debug_level >= 2):
            rospy.logdebug("{}: Computed shape context".format(fname))
            for (index, (contour, contour_SC)) in enumerate(contours_SC):
                contour_img = np.zeros((cv_img.shape[0], cv_img.shape[1]))
                cv2.drawContours(contour_img, contour, -1, 255)
                cv2.imshow("contour_{}".format(index), contour_img)
            cv2.waitKey(0)

        ''' Iterate over all contours in the image and find the best contour match in
                    the DB. Push the contour into a min-heap according to the difference between
                    the contour and its best match from the DB. '''
        sc = cupRec.SC.SC()
        contour_list = []
        for (contour_idx, (contour, contour_SC)) in enumerate(contours_SC):

            rospy.logdebug("{}: Finding best match for contour_{}. Stand by...".format(fname, contour_idx))
            match_cost = sys.maxint
            match_DB_contour = self.shapecontext_DB[0][0]
            DB_match_idx = 0
            for (index, (DB_contour, DB_contour_SC)) in enumerate(self.shapecontext_DB):
                ''' Compute the difference between the contour and the DB contour '''
                contour_cost, indices = sc.diff(DB_contour_SC, contour_SC)
                if (contour_cost < match_cost):
                    match_cost = contour_cost
                    match_DB_contour = DB_contour
                    DB_match_idx = index

            if (self.debug_level >= 1):
                rospy.logdebug("{}: Min Cost = {}".format(fname, match_cost))
                rospy.logdebug("{}: DB contour match = {}".format(fname, DB_match_idx))

            if (self.debug_level >= 2):
                match_contour_img = np.zeros((cv_img.shape[0], cv_img.shape[1]))
                cv2.drawContours(match_contour_img, match_DB_contour, -1, 255)
                cv2.imshow("matched contour", match_contour_img)
                cv2.waitKey(0)

            ''' All DB contours iterated, push contour onto heap by best match '''
            contour_list.append((match_cost, contour, match_DB_contour))
        return contour_list

    def cupRec_aggregation_cb(self, snapshot_data):
        fname = self.cupRec_aggregation_cb.__name__

        '''
        Takes a snapshot data object, filters the image in it according to the color filtering parameters
        in the message and computes the shapecontext of every contour (actually, every contour not
        contained inside another contour) in the contained image and compares the contours' similarity
        to the local database.

        :param snapshot_data: Data from /cupinator/cup_recognizer/snapshot topic (message type snapshotData)
        :type snapshot_data: AggregationData
        '''

        if (self.debug_level >= 1):
            start = time.time()
        else:
            start = 0
        try:
            cv_img = cv_bridge.CvBridge().imgmsg_to_cv2(snapshot_data.Image)
        except cv_bridge.CvBridgeError, cv_bridge_except:
            rospy.logerr("{}: Failed to convert ROS image message to CvMat\n{}".format(fname, cv_bridge_except))
            return

        contour_list = self.compute_contours_shapecontext_cost(cv_img)

        self.lock.acquire()

        for (cost, contour, DB_contour) in contour_list:
            heapq.heappush(self.contour_heap, (cost, contour, DB_contour))

        self.lock.release()

        if (self.debug_level >= 1):
            end = time.time()
            rospy.logdebug("{}: time = {}".format(fname, end - start))

    def cupRec_aggregate_query_cb(self, cupRec_query_request):
        fname = self.cupRec_single_query_cb.__name__
        """

        :param cupRec_query_request: Request message
        :type cupRec_query_request: CupRecQueryRequest

        :return: CupRecQueryResponse
        """

        rospy.logdebug("{}: Acquiring match heap lock".format(fname))
        self.lock.acquire()

        response = CupRecQueryResponse()
        response_match = self.contour_heap[0]

        response.match_cost = response_match[0]
        response.point_count = len(response_match[1])
        rospy.logdebug("{}: The best match costs {} and has {} points".
                       format(fname, response.match_cost, response.point_count))
        response.x_coords = response_match[1][:, 0, 0]
        response.x_coords = response_match[1][:, 0, 1]

        if (cupRec_query_request.flush):
            rospy.logdebug("{}: flushing match heap".format(fname))
            self.contour_heap = []

        rospy.logdebug("{}: Releasing match heap lock".format(fname))
        self.lock.release()

        return response

    def subscribe_to_snapshots(self):
        rospy.Subscriber("/cupinator/cup_recognizer/image_aggregate", AggregationData, self.cupRec_aggregation_cb,
                         queue_size=12)
        rospy.Service("/cupinator/cup_recognizer/single_cupRec_query", SingleCupRecQuery, self.cupRec_single_query_cb)
        rospy.Service("/cupinator/cup_recognizer/cupRec_query_aggregate", CupRecQuery, self.cupRec_aggregate_query_cb)


if __name__ == '__main__':
    try:
        fname = "main"
        rospy.init_node("CupRec", anonymous=False, log_level=rospy.DEBUG)
        cupRecognizer = ShapeContextCupRec()
        rospy.loginfo("{}: Initializing node".format(fname))
        rospy.loginfo("{}: Starting subscriber".format(fname))
        cupRecognizer.subscribe_to_snapshots()

        rospy.spin()
    except rospy.ROSInterruptException, e:
        print e