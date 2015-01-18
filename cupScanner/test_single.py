from utils import get_points_from_img
from SC import SC
import time
import numpy as np
import cv2

if __name__ == '__main__':

    sc = SC()
    samples = 100

    start = time.time()
    points1, t1 = get_points_from_img('cupT.bmp', simpleto=samples)
    points2, t2 = get_points_from_img('cupB.bmp', simpleto=samples)

    src = cv2.imread('cup.jpg')
    dst = cv2.Canny(src, 50, 150)

    #contours, hierarchy = cv2.findContours(src, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    #dest2 = np.array((2000,2000,2000), np.uint8)
    print type(src), src.shape, src.size
    blue = src[:, :, 0]
    print type(blue), blue.shape, blue.size
    green = src[:, :, 1]
    red = src[:, :, 2]

    for rowIdx in range(0, blue.shape[0]):
        for colIdx in range(0, blue.shape[1]):
            if (green[rowIdx, colIdx] * 1.5 > blue[rowIdx, colIdx]) | (red[rowIdx, colIdx] * 1.5 > blue[rowIdx, colIdx]):
                blue[rowIdx, colIdx] = 0
            else:
                blue[rowIdx, colIdx] = 255

    cv2.imshow('all', src)
    cv2.waitKey(0)

    cv2.imshow('blue', blue)
    cv2.waitKey(0)

    #P = sc.compute(points1)
    #Q = sc.compute(points2)
    #COST, indexes = sc.diff(Q, P)
    end = time.time()

    print "time:", end - start
    #print "cost:", COST
