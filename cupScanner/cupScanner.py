from utils import get_points_from_img
from SC import SC
import time
from math import sin, cos, sqrt, pi
import math
import numpy as np
import cv2
import cv


def get_points_from_b_and_w_img(bw_img, sampleto, low_th=50, high_th=200):

    """
    :param bw_img: (ndarray) Image to get contour sample from
    :param sampleto: (int) Number of samples
    :return: (array) A 2D array of coordinates (x,y) sampled from the contour
    """
    points = []
    if (bw_img[0,0] < low_th):
        state = 0
    else:
        state = 1
    for x in xrange(0, bw_img.shape[0]):
        for y in xrange(0, bw_img.shape[1]):
            c = bw_img.item((x, y))
            if ((state == 0) & (c >= high_th)) |\
                    ((state ==1) & (c <= low_th)):
                state = 1 - state
                points.append((x, y))
                print "(", x, ", ", y,")"

    r = 2
    w, h = bw_img.shape
    while len(points) > sampleto:
        newpoints = points
        xr = xrange(0,w,r)
        yr = xrange(0,h,r)
        for p in points:
            if p[0] not in xr and p[1] not in yr:
                newpoints.remove(p)
                if len(points) <= sampleto:
                    return points
        r += 1
    '''T = np.zeros((sampleto,1))
    for i,(x,y) in enumerate(points):
        T[i] = math.atan2(py[y,x],px[y,x])+pi/2;
    '''
    return points #,np.asmatrix(T)


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

    pts = get_points_from_b_and_w_img(src[:, :, 1], 150)
    pts_test = get_points_from_b_and_w_img(src_test[:, :, 1], 150)

    cv2.imshow('green2', src[:, :, 1])

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
