import cv2
import numpy as np
from DrawMatches import drawMatches

# img = cv2.imread('imgs/img.jpg')
# cv2.imshow('image', img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

cap = cv2.VideoCapture('imgs/test.avi')
fast = cv2.FastFeatureDetector()
surf = cv2.SURF(400);
orb = cv2.ORB()

frames = []
key_points = []
descrs = []
cap.set(cv2.cv.CV_CAP_PROP_POS_FRAMES,1000)

for i in range(2):
    ret, frame = cap.read()
    frames.append(frame)
    # key_point = fast.detect(frame,None)
    #det = surf.detectAndCompute(frame,None)
    (kp, descr) = orb.detectAndCompute(frame, None)
    key_points.append(kp)
    descrs.append(descr)

bf = cv2.BFMatcher()
matches = bf.match(descrs[0],descrs[1])

# Sort the matches based on distance.  Least distance
# is better
matches = sorted(matches, key=lambda val: val.distance)

# res = drawMatches(frames[0], key_points[0], frames[1], key_points[1], matches[:50])

points0 = np.array([key_points[0][match.queryIdx].pt for match in matches])
points1 = np.array([key_points[1][match.queryIdx].pt for match in matches])

F = cv2.findFundamentalMat(points0, points1, cv2.FM_RANSAC, 0.1, 0.99, 3)
F = F[0]


U,D,V = cv2.SVDecomp(np.mat(F))
print(np.mat(V))

#defining FLANN
# index_params = dict(algorithm = 0, trees = 5)
# search_params = dict(checks = 50)
# flann = cv2.FlannBasedMatcher(index_params, search_params)
#
# matches = []
# for i in range(1):
#     match = flann.knnMatch(features[i][1],features[i+1][1],k=2)
#     matches.append(match)
#
# print match
#
# src_pts = np.float32([ features[0][0][m[0][0].queryIdx].pt for m in matches ]).reshape(-1,1,2)
# dst_pts = np.float32([ features[1][0][m[0][0].trainIdx].pt for m in matches ]).reshape(-1,1,2)
#
# M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
# matchesMask = mask.ravel().tolist()
#
# h,w = frame.shape
# pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
# dst = cv2.perspectiveTransform(pts,M)
#
# img2 = cv2.polylines(matches,[np.int32(dst)],True,255,3, cv2.LINE_AA)
#
#
#
#
# #show image
# img2 = cv2.drawKeypoints(frame, kp, color=(255,0,0))
# cv2.imshow('frame',img2)
# cv2.waitKey(1)
# cv2.destroyAllWindows()

