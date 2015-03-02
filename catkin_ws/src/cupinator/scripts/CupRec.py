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
from cupinator.msg import SnapshotData
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
    main_channel = 0
    main_lower_th = 0
    secondary_factor = 0
    sample_size = 0

    lock = Lock()
    shapecontext_DB = []
    contour_heap = []

    def load_shapecontext_DB(self):
        #debug
        start_time = time.time()
        #end debug
        #TODO: Store computed SC on disk to reduce startup time
        DB_path = os.path.join(os.path.dirname(__file__), "cupRec/DB/")
        for DB_file in os.listdir(DB_path):
            if DB_file.endswith(".jpg") | DB_file.endswith(".png") | DB_file.endswith(".bmp"):
                DB_img = cv2.imread(DB_path + DB_file)
                shapecontexts = self.compute_shapecontext(DB_img)
                for (img_contour, contour_SC) in shapecontexts:
                    self.shapecontext_DB.append((img_contour, contour_SC))

        if __debug__:
            end_time = time.time()
            rospy.logdebug("cupinator/cupRec: DB load time = %s" % str(end_time - start_time))
        #end debug

    def __init__(self, main_channel=1, main_lower_th=50, secondary_factor=2, sample_size=100):
        self.main_channel = main_channel
        self.main_lower_th = main_lower_th
        self.secondary_factor = secondary_factor
        self.sample_size = sample_size
        self.load_shapecontext_DB()

    def sample_contours_from_binary_img(self, bw_img, sample_size, low_th=50, high_th=200):

        """
        :param bw_img: (ndarray) Image to get contour sample from
        :param sample_size: (int) Maximal number of samples
        :return: (array) A 2D array of coordinates (x,y) sampled from the contour
        """
        contour_img = cv2.Canny(bw_img, low_th, high_th)
        contours, dont_care = cv2.findContours(contour_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        contours_samples = []

        for contour in contours:
            contour_samples_count = sample_size
            contour_samples = np.ndarray((contour_samples_count, 1, 2), contours[0].dtype)

            marker_step = max((float)(len(contour)) / (float)(sample_size), 1)
            contour_marker = 0
            contour_idx = 0
            for sample_idx in xrange(0, contour_samples_count):
                if (contour_idx >= len(contour)):
                    contour_marker = 0
                    contour_idx = 0
                contour_samples[sample_idx, :, :] = contour[contour_idx].copy()

                contour_marker += marker_step
                contour_idx = round(contour_marker)

            contours_samples.append(contour_samples)

        return contours_samples

    def compute_shapecontext(self, cv_img):

        """
        :rtype : List of (Contour, ShapeContext) elements
        :param cv_img: Input RGB (or any 3 channel ordering, like BGR) image
        """

        second_channel = (self.main_channel + 1) % 3
        third_channel = (self.main_channel + 2) % 3

        for rowIdx in xrange(0, cv_img.shape[0]):
            for colIdx in xrange(0, cv_img.shape[1]):
                main_channel_val = cv_img.item((rowIdx, colIdx, self.main_channel))
                second_channel_val = cv_img.item((rowIdx, colIdx, second_channel))
                third_channel_val = cv_img.item((rowIdx, colIdx, third_channel))
                if (second_channel_val * self.secondary_factor > main_channel_val) | \
                        (third_channel_val * self.secondary_factor > main_channel_val) | \
                        (main_channel_val < self.main_lower_th):
                    cv_img.itemset((rowIdx, colIdx, self.main_channel), 0)
                else:
                    cv_img.itemset((rowIdx, colIdx, self.main_channel), 255)

        input_samples = self.sample_contours_from_binary_img(cv_img[:, :, self.main_channel], self.sample_size)

        sc = cupRec.SC.SC()

        contours_SC = []
        for input_contour_samples in input_samples:
            contour_SC = sc.compute(input_contour_samples[:, 0, :])
            contours_SC.append((input_contour_samples, contour_SC))

        return contours_SC

    def shapecontext_aggregation_cb(self, snapshot_data):
        """
        Takes a snapshot data object, filters the image in it according to the color filtering parameters
        in the message and computes the shapecontext of every contour (actually, every contour not
        contained inside another contour) in the contained image and compares the contours' similarity
        to the local database.

        :param snapshot_data: Data from /cupinator/cup_recognizer/snapshot topic (message type snapshotData)
        :type snapshot_data: SnapshotData
        """

        start = time.time()
        try:
            cv_img = cv_bridge.CvBridge().imgmsg_to_cv2(snapshot_data.Image)
        except cv_bridge.CvBridgeError, cv_bridge_except:
            rospy.logerr("Failed to convert ROS image message to CvMat\n%s", str(cv_bridge_except))
            return

        if __debug__:
            cv2.imshow('all', cv_img)
            cv2.waitKey(0)
        #end debug

        ''' Compute the shapecontext for all the contours in the image.
            Note: Ignores contours nested inside other contours '''
        contours_SC = self.compute_shapecontext(cv_img)

        if __debug__:
            for (contour, contour_SC) in contours_SC:
                contour_img = np.zeros((cv_img.shape[0], cv_img.shape[1]))
                cv2.drawContours(contour_img, contour, -1, 255)
                cv2.imshow("contour", contour_img)
            cv2.waitKey(0)
        #end debug

        ''' Iterate over all contours in the image and find the best contour match in
            the DB. Push the contour into a min-heap according to the difference between
            the contour and its best match from the DB. '''
        sc = cupRec.SC.SC()

        contour_list = []
        for (contour, contour_SC) in contours_SC:

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

            if __debug__:
                rospy.logdebug("Min Cost = %s" % str(match_cost))
                rospy.logdebug("DB contour match = %s" % str(DB_match_idx))

                match_contour_img = np.zeros((cv_img.shape[0], cv_img.shape[1]))
                cv2.drawContours(match_contour_img, match_DB_contour, -1, 255)
                cv2.imshow("matched contour", match_contour_img)
                cv2.waitKey(0)
            #end debug

            ''' All DB contours iterated, push contour onto heap by best match '''
            contour_list.append((match_cost, contour, match_DB_contour))

        self.lock.acquire()

        for (cost, contour, DB_contour) in contour_list:
            heapq.heappush(self.contour_heap, (cost, contour, DB_contour))

        self.lock.release()

        if __debug__:
            end = time.time()
            rospy.logdebug("time = %s" % str(end - start))
        #end debug

    def subscribe_to_snapshots(self):
        rospy.Subscriber("/cupinator/cup_recognizer/snapshot", SnapshotData, self.shapecontext_aggregation_cb)


if __name__ == '__main__':
    try:
        rospy.init_node("CupRec", anonymous=False, log_level=rospy.DEBUG)
        rospy.loginfo("Loading DB")
        cupRecognizer = ShapeContextCupRec()
        rospy.loginfo("Initializing node")
        rospy.loginfo("Starting subscriber")
        cupRecognizer.subscribe_to_snapshots()

        rospy.spin()
    except rospy.ROSInterruptException, e:
        print e