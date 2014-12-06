__author__ = 'niclas'

"""
Camera Calibration using the OpenCV functionalities.
Checkerboard calibration is used.
Based on the code from the tutorial: http://docs.opencv.org/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
and http://docs.opencv.org/trunk/doc/py_tutorials/py_calib3d/py_calibration/py_calibration.html#calibration
"""

import numpy as np
import cv2
from helpers import get_files_with_ending
from debug import debug

def do_calibration(images_names, show_corners=False):
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    row = 9
    col=6

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((row*col,3), np.float32)
    objp[:,:2] = np.mgrid[0:row,0:col].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.


    images = [cv2.imread(fname) for fname in images_names]
    grays = [cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) for img in images]
    for i in range(len(grays)):
        gray = grays[i]
        img = images[i]
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (row,col),None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners)
            if show_corners:
                # Draw and display the corners
                cv2.drawChessboardCorners(img, (row,col), corners,ret)
                cv2.imshow('img',img)
                cv2.waitKey(500)

    if show_corners:
        cv2.destroyAllWindows()

    gray = grays[0]
    img = images[0]

    #get camera matrix
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    h,  w = img.shape[:2]
    camera_matrix, roi =cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
    return (mtx, dist, camera_matrix, roi)


def undistort((mtx, dist, camera_matrix, roi), imgs, dest_imgs, crop=True):

    for i in range(len(imgs)):
        img = cv2.imread(imgs[i])

        undist_img = cv2.undistort(img, mtx, dist, None, camera_matrix)

        # crop the image
        if crop:
            x,y,w,h = roi
            undist_img = undist_img[y:y+h, x:x+w]

        #save new image
        cv2.imwrite(dest_imgs[i],undist_img)

    # mean_error = 0
    # for i in xrange(len(objpoints)):
    #     imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    #     error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    #     mean_error += error
    #
    # print "total error: ", mean_error/len(objpoints)

def calibrate(calib_img_dir, img_dir, dest_dir, img_endings, parallel=False, crop=True):

    calib_imgs = get_files_with_ending(calib_img_dir, img_endings)
    imgs = get_files_with_ending(img_dir, img_endings)
    dest_imgs = []
    for img in imgs:
        name = img.split('/')
        name = dest_dir+name[-1]
        dest_imgs.append(name)

    #check whether there are objects in the lists
    if len(calib_imgs)==0 or len(imgs)==0 or len(dest_imgs)==0:
        debug(2,'Calibration can not be executed. The specified folders are invalid or don\'t contain images.')
        return

    calib_para = do_calibration(calib_imgs)

    if parallel:
        pass #TODO: implement
    else:
        undistort(calib_para,imgs=imgs, dest_imgs=dest_imgs, crop=crop)


calibrate('../vid_calib/', '../roaming/vid_imgs/', '../roaming/vid_imgs/calib_', ['jpg','JPG','jpeg','JPEG'], crop=False)