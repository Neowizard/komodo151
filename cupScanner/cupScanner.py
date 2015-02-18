import sys
import time
import numpy as np
import cv2

from SC import SC


def sample_contours_from_binary_img(bw_img, sample_size, low_th=50, high_th=200):

    """
    :param bw_img: (ndarray) Image to get contour sample from
    :param sample_size: (int) Maximal number of samples
    :return: (array) A 2D array of coordinates (x,y) sampled from the contour
    """
    contour_img = cv2.Canny(bw_img, low_th, high_th)
    contours, dont_care = cv2.findContours(contour_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    contours_samples = []

    for contour in contours:
        contour_samples_count = min(len(contour), sample_size)
        contour_samples_count = sample_size
        contour_samples = np.ndarray((contour_samples_count, 1, 2), contours[0].dtype)

        marker_step = max((float)(len(contour)) / (float)(sample_size), 1)
        contour_marker = 0
        contour_idx = 0
        for sample_idx in xrange(0, contour_samples_count):
            if (contour_idx >= len(contour)):
                #break
                contour_marker = 0
                contour_idx = 0
            contour_samples[sample_idx, :, :] = contour[contour_idx].copy()

            contour_marker += marker_step
            contour_idx = round(contour_marker)

        contours_samples.append(contour_samples)

    '''
    contour_marker = 0
    for contour in contours:
        while 1:
            if (round(contour_marker) >= len(contour)):
                break
            contour_samples.append(contour[round(contour_marker)][0])
            contour_marker += max(marker_step, 1)
        contour_marker -= len(contour)
    '''

    return contours_samples


if __name__ == '__main__':

    src = cv2.imread('green1.jpg')

    print type(src), src.shape, src.size
    blue = src[:, :, 0]
    print type(blue), blue.shape, blue.size
    green = src[:, :, 1]
    red = src[:, :, 2]

    cv2.imshow('all', src)
    cv2.waitKey(0)

    start = time.time()

    for rowIdx in xrange(0, src.shape[0]):
        for colIdx in xrange(0, src.shape[1]):
            blue = src.item((rowIdx, colIdx, 0))
            green = src.item((rowIdx, colIdx, 1))
            red = src.item((rowIdx, colIdx, 2))
            if (blue * 2 > green) | (red * 2 > green) | (green < 50):
                src.itemset((rowIdx, colIdx, 1), 0)
            else:
                src.itemset((rowIdx, colIdx, 1), 255)

    src_test = cv2.imread('green_test.jpg')

    '''
    pts, hierarchy = cv2.findContours(dst, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    pts_test, h_crap = cv2.findContours(dst_test, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    pts = pts[0][:, 0, :]
    pts_test = pts_test[0][:, 0, :]

    contour = np.ndarray((100, 2))
    contour_test = np.ndarray((100, 2))

    ratio = min(math.floor(pts.shape[0] / 100), 1)
    for point in xrange(0, pts.shape[0], ratio):
        contour[point] = pts[point]

    ratio = math.min(math.floor(pts_test.shape[0] / 100), 1)
    for point in xrange(0, pts_test.shape[0], ratio):
        contour_test[point] = pts_test[point]
    '''

    input_samples = sample_contours_from_binary_img(src[:, :, 1], 100)
    test_contour_samples = sample_contours_from_binary_img(src_test[:, :, 1], 100)

    '''
    #debug

    test_contours_img = np.zeros(src_test.shape)
    src_contours_img = np.zeros(src.shape)

    cv2.drawContours(src_contours_img, contour_samples, -1, (255, 255, 255))
    cv2.drawContours(test_contours_img, test_contour_samples, -1, (255, 255, 255))

    cv2.imshow("contours_test", test_contours_img)
    cv2.imshow("contours", src_contours_img)
    cv2.waitKey(0)

    #end debug
    '''

    sc = SC()
    test_contour_SC = sc.compute(test_contour_samples[0][:, 0, :])

    input_contours_SC = []
    min_cost = sys.maxint
    min_cost_contour = input_samples[0]
    for input_contour_samples in input_samples:
        contour_SC = sc.compute(input_contour_samples[:, 0, :])
        contour_cost, indices = sc.diff(test_contour_SC, contour_SC)
        input_contours_SC.append((contour_SC, contour_cost))
        print "Cost = ", contour_cost

        if (contour_cost < min_cost):
            min_cost = contour_cost
            min_cost_contour = input_contour_samples

    print "Min Cost = ", min_cost

    min_cost_contour_img = np.zeros(src.shape)
    cv2.drawContours(min_cost_contour_img, min_cost_contour, -1, (255, 255, 255))
    end = time.time()

    print "time:", end - start

    cv2.imshow("min_cost contour", min_cost_contour_img)
    cv2.waitKey(0)

    #print "cost:", COST
