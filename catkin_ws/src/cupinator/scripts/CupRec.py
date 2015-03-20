#!/usr/bin/env python
import heapq
import sys
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
    debug_level = 4

    lock = Lock()
    shapecontext_DB = []
    contour_heap = []

    def __init__(self, sample_size=100):
        if (self.debug_level >= 1):
            rospy_debug_level = rospy.DEBUG
        else:
            rospy_debug_level = None
        self.sample_size = sample_size
        self.load_shapecontext_DB()

    def load_shapecontext_DB(self):
        fname = "{}::{}".format(self.__class__.__name__,  self.load_shapecontext_DB.__name__)
        rospy.loginfo("Loading DB")
        start_time = time.time()

        # TODO: Store computed SC on disk to reduce startup time
        DB_path = os.path.join(os.path.dirname(__file__), "cupRec/DB/")
        for DB_file in os.listdir(DB_path):
            if DB_file.lower().endswith(".jpg") | DB_file.lower().endswith(".png") | DB_file.lower().endswith(".bmp"):
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

        contours, _ = cv2.findContours(contour_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if (self.debug_level >= 4):
            print "contours count = ", len(contours)
        lst_contours_samples = []

        for (contour_idx, contour) in enumerate(contours):
            if (len(contour) < sample_size):
                continue
            if (self.debug_level >= 4):
                print "contour #", contour_idx, " length = ", len(contour)
            contour_samples = np.ndarray((sample_size, 1, 2), contours[0].dtype)

            marker_step = (float)(len(contour)) / (float)(sample_size)
            if (marker_step < 1):
                print "{}: ----- ERROR ----- marker step < 1 "
            contour_marker = 0
            contour_sample_idx = 0
            for sample_idx in xrange(0, sample_size):
                if (contour_sample_idx >= len(contour)):
                    break
                contour_samples[sample_idx, :, :] = contour[contour_sample_idx]

                contour_marker += marker_step
                contour_sample_idx = round(contour_marker)
            lst_contours_samples.append(contour_samples)

        if (self.debug_level >= 4):
            print "contours:\n", contours
            print "sampled contours:\n", lst_contours_samples

        return lst_contours_samples

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

    def cupRec_single_query(self, cv_img):
        fname = "{}::{}".format(self.__class__.__name__,  self.cupRec_single_query.__name__)

        """

        :param single_snapshot_query:
        :type single_snapshot_query: numpy.ndarray
        """

        rospy.logdebug("{}: Starting single query cup recognition...".format(fname))

        if (self.debug_level >= 1):
            start = time.time()
        else:
            start = 0

        rospy.logdebug("{}: Computing shape contexts for contours in image".format(fname))
        contour_match_list = self.compute_contours_match_cost(cv_img)

        min_cost = sys.maxint
        min_cost_idx = -1
        for (match_idx, (match_cost, contour, DB_contour)) in enumerate(contour_match_list):
            if (match_cost < min_cost):
                min_cost = match_cost
                min_cost_idx = match_idx
                rospy.logdebug("{}: Setting best-cost contour to contour_{}".format(fname, match_idx))

        if (self.debug_level >= 1):
            end = time.time()
            rospy.logdebug("{}: Query time = {}".format(fname, end - start))
        rospy.logdebug("{}: Sending response".format(fname))

        return contour_match_list[min_cost_idx]

    def compute_contours_match_cost(self, cv_img):
        fname = "{}::{}".format(self.__class__.__name__,  self.compute_contours_match_cost.__name__)
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
        rospy.logdebug("{}: Computing shape context for image".format(fname))
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

            rospy.loginfo("{}: Finding best match for contour_{}. Stand by...".format(fname, contour_idx))
            match_cost = sys.maxint
            match_DB_contour = self.shapecontext_DB[0][0]
            DB_match_idx = 0
            for (index, (DB_contour, DB_contour_SC)) in enumerate(self.shapecontext_DB):
                ''' Compute the difference between the contour and the DB contour '''
                rospy.logdebug("{}: Computing match cost to DB contour #{}".format(fname, index))
                contour_cost, indices = sc.diff(DB_contour_SC, contour_SC)
                rospy.logdebug("{}: Match cost = {}".format(fname, contour_cost))

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

    def aggregate_image(self, image_data):
        fname = "{}::{}".format(self.__class__.__name__,  self.aggregate_image.__name__)

        '''
        Takes a snapshot data object, filters the image in it according to the color filtering parameters
        in the message and computes the shapecontext of every contour (actually, every contour not
        contained inside another contour) in the contained image and compares the contours' similarity
        to the local database.

        :param snapshot_data: Data from /cupinator/cup_recognizer/snapshot topic (message type snapshotData)
        :type snapshot_data: (np.ndarray, key)
        '''

        if (self.debug_level >= 1):
            start = time.time()
        else:
            start = 0
        cv_img = image_data[0]

        contour_list = self.compute_contours_match_cost(cv_img)

        self.lock.acquire()

        for (cost, contour, DB_contour) in contour_list:
            heapq.heappush(self.contour_heap, (cost, contour, image_data[1], DB_contour))

        self.lock.release()

        if (self.debug_level >= 1):
            end = time.time()
            rospy.logdebug("{}: time = {}".format(fname, end - start))

    def cupRec_aggregate_query(self, flush_heap=False):
        """

        :param flush_heap: boolean indicating if we need to reset the heap storing all the computed SCs
        :return: (cost, contour, key, DB_contour) with the lower cost in the contour_heap
        """
        fname = "{}::{}".format(self.__class__.__name__,  self.cupRec_aggregate_query.__name__)

        rospy.logdebug("{}: Acquiring match heap lock".format(fname))
        self.lock.acquire()

        optimal_match = self.contour_heap[0]

        if (flush_heap):
            rospy.logdebug("{}: flushing match heap".format(fname))
            self.contour_heap = []

        rospy.logdebug("{}: Releasing match heap lock".format(fname))
        self.lock.release()

        return optimal_match


if __name__ == '__main__':
    try:
        main_name = "{}::{}".format("CupRec.py",  "main")

        cupRecognizer = ShapeContextCupRec()

        test_img_file_name = "red_test.jpg"
        mock_path = os.path.join(os.path.dirname(__file__), "MockCamInput/")
        rospy.logdebug("{}: loading image from {}".format(main_name, mock_path + test_img_file_name))
        cap_img = cv2.imread(mock_path + test_img_file_name)
        rospy.logdebug("{}: Sending {} to cupRec".format(main_name, test_img_file_name))
        best_match = cupRecognizer.cupRec_single_query(cap_img)
        rospy.logdebug("{}: {} cost = {}".format(main_name, test_img_file_name, best_match[0]))

    except rospy.ROSInterruptException, e:
        print e