import cv2

# img = cv2.imread('imgs/img.jpg')
# cv2.imshow('image', img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

cap = cv2.VideoCapture('imgs/test.avi')
fast = cv2.FastFeatureDetector()
surf = cv2.SURF(400);

while(cap.isOpened()):
    ret, frame = cap.read()
    # find and draw the keypoints
    kp = fast.detect(frame,None)
    # kp, des = surf.detectAndCompute(frame,None)

    img2 = cv2.drawKeypoints(frame, kp, color=(255,0,0))

    # Print all default params
    print "Threshold: ", fast.getInt('threshold')
    print "nonmaxSuppression: ", fast.getBool('nonmaxSuppression')
    print "Total Keypoints with nonmaxSuppression: ", len(kp)


    cv2.imshow('frame',img2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()