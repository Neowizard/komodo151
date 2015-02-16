import time
import numpy as np
import cv2

from SC import SC


def sample_contours_from_binary_img(bw_img, sample_size, low_th=50, high_th=200):

    """
    :param bw_img: (ndarray) Image to get contour sample from
    :param sample_size: (int) Number of samples
    :return: (array) A 2D array of coordinates (x,y) sampled from the contour
    """
    contour_img = cv2.Canny(bw_img, low_th, high_th)
    contours, dont_care = cv2.findContours(contour_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    contours_samples = []

    for contour in contours:
        contour_samples = []
        marker_step = max((float)(len(contour)) / (float)(sample_size), 1)
        contour_marker = 0
        contour_idx = 0
        while (contour_idx < len(contour)):
            contour_samples.append(contour[contour_idx][0])
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

    samples = 100

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

    pts = sample_contours_from_binary_img(src[:, :, 1], 100)
    pts_test = sample_contours_from_binary_img(src_test[:, :, 1], 100)

    #debug
    test_contours_img = np.zeros(src_test.shape)
    src_contours_img = np.zeros(src.shape)
    pts_ndarr = np.asarray(pts)
    pts_test_ndarr = np.asarray(pts_test)
    cv2.drawContours(src_contours_img, pts_ndarr, -1, (255, 255, 255))
    cv2.drawContours(test_contours_img, pts_test_ndarr, -1, (255, 255, 255))

    cv2.imshow("contours_test", test_contours_img)
    cv2.imshow("contours", src_contours_img)
    cv2.waitKey(0)

    '''
    f = open("test.txt", "w")
    f.write(repr(pts))
    f.write('\n')
    f.write(repr(pts_test))
    f.close()
    '''
    #end debug

    sc = SC()
    P = sc.compute(pts)
    Q = sc.compute(pts_test)
    COST, indexes = sc.diff(Q, P)

    print "Cost = ", COST

    end = time.time()

    print "time:", end - start
    #Q = sc.compute(points2)
    cv2.waitKey(0)

    #print "cost:", COST
