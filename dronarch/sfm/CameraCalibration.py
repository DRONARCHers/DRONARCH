from dronarch.helpers.parallel_exe import parallel_exe

__author__ = 'niclas'

"""
Camera Calibration using the OpenCV functionalities.
Checkerboard calibration is used.
Based on the code from the tutorial: http://docs.opencv.org/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
and http://docs.opencv.org/trunk/doc/py_tutorials/py_calib3d/py_calibration/py_calibration.html#calibration
"""

import numpy as np
import cv2
from dronarch.helpers.helpers import get_files_with_ending,timestamp
from dronarch.helpers.debug import debug


def do_calibration(images_names, show_corners=False):
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    row = 9
    col= 6

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
    if not len(imgs) == len(dest_imgs):
        debug(2,'Could not undistort, lists imgs and dest_imgs have different length')
        return

    #parallel execution needs reforming of parameters
    params = [[imgs[i], dest_imgs[i], mtx, dist, camera_matrix, roi, crop] for i in range(len(imgs))]
    parallel_exe(undistort_parallel, params)



def undistort_parallel(params):
    #extract parameters from tupple
    img_name = params[0]
    dest_img = params[1]
    mtx = params[2]
    dist = params[3]
    camera_matrix = params[4]
    roi = params[5]
    crop = params[6]

    #read image
    img = cv2.imread(img_name)

    #undistort image
    undist_img = cv2.undistort(img, mtx, dist, None, camera_matrix)
    # cv2.imshow('image', undist_img)
    # cv2.waitKey(5000)
    # cv2.destroyAllWindows()


    # crop the image
    if crop:
        margin = 0
        x,y,w,h = roi
        undist_img = undist_img[y:y+h, x+margin:x+w-margin]

    #save new image
    cv2.imwrite(dest_img,undist_img)


def calibrate(calib_img_dir, img_dir, dest_dir, calib_dest_dir, img_endings, crop=False):
    debug(0, 'Estimating camera calibration using images from directory ', calib_img_dir)
    calib_imgs = get_files_with_ending(calib_img_dir, img_endings)
    imgs = get_files_with_ending(img_dir, img_endings)

    #create filenames for undistort input images
    dest_imgs = []
    for img in imgs:
        name = img.split('/')
        name = dest_dir+name[-1]
        dest_imgs.append(name)

    #create filenames for undistort calibration images
    dest_calib_img = []
    for calib_img in calib_imgs:
        name = calib_img.split('/')
        name = calib_dest_dir+name[-1]
        dest_calib_img.append(name)


    #check whether there are objects in the lists
    if len(calib_imgs)==0 or len(imgs)==0 or len(dest_imgs)==0:
        debug(2,'Calibration can not be executed. The specified folders are invalid or don\'t contain images.')
        return
    calib_para = do_calibration(calib_imgs, show_corners=False)
    write_to_calib_file('../../config/first_calib.txt', calib_para)

    #Undistort input images
    debug(0, 'Undistort images from directory ', img_dir)
    undistort(calib_para,imgs=imgs, dest_imgs=dest_imgs, crop=crop)

    # show_image(imgs[0])

    #undistort calibration images
    undistort(calib_para,imgs=calib_imgs, dest_imgs=dest_calib_img, crop=crop)

    debug(0, 'Calibration done.')
    timestamp()
    return dest_calib_img

def calibrate_two_times(calib_img_dir, img_dir, dest_dir, img_endings, calib_file_path, calib_dest_dir, crop=True):
    #Do first calibration and undistort all images
    calib_imgs = calibrate(calib_img_dir=calib_img_dir, img_dir=img_dir, dest_dir=dest_dir, img_endings=img_endings, calib_dest_dir=calib_dest_dir, crop=crop)

    debug(0, 'Do second calibration to generate calibration file for bundler')
    calib_para = do_calibration(calib_imgs)

    write_to_calib_file(calib_file_path=calib_file_path, calib_para=calib_para)

def write_to_calib_file(calib_file_path, calib_para):
    with open(calib_file_path, 'w+') as file:
        dist = calib_para[1]
        camera_matrix = calib_para[2]
        matrix_str = ' '.join([str(camera_matrix[i,j]) for i in range(3) for j in range(3)])
        dist_str = ' '.join([str(dist[0,i]) for i in range(dist.size)])
        file.write('1\n')
        file.write(matrix_str+'\n')
        file.write(dist_str)

    debug(0, 'Saved calibration file: ', calib_file_path)

def show_image(image_file):
    img = cv2.imread(image_file)
    cv2.imshow('image', img)
    cv2.waitKey(5000)
    cv2.destroyAllWindows()

if __name__ =='__main__':
    calibrate('../../vid_calib/', '../../roaming/vid_imgs/', '../../roaming/vid_imgs/calib_', ['jpg','JPG','jpeg','JPEG'], crop=False)


    # mean_error = 0
    # for i in xrange(len(objpoints)):
    #     imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    #     error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    #     mean_error += error
    #
    # print "total error: ", mean_error/len(objpoints)