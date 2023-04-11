import numpy as np
import cv2
import glob
import sys
from time import sleep

# termination criteria
# 10 iterations, 0.01 accuracy
criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

n_corners = (9,6)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((n_corners[0]*n_corners[1],3), np.float32)
objp[:,:2] = np.mgrid[0:n_corners[0],0:n_corners[1]].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
imgpoints = [] # 2d points in image plane.
filenames = [] # filenames of included images

folder = 'images'
images = glob.glob('./'+str(folder)+'/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    print(str(fname))
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, n_corners,None,cv2.CALIB_CB_FAST_CHECK)

    # If found, refine and display image points
    if ret == True:
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

        # Draw and display the corners
        scaler = 0.3
        res = tuple( [int(k*scaler) for k in img.shape[1::-1]] )
        img = cv2.drawChessboardCorners(cv2.resize( img , res), n_corners, np.array(corners2)*scaler,ret) #resize
        cv2.imshow(fname, img) #display image
        cv2.moveWindow(fname, 50, 50) #window position

        #if user approves, add object points
        key = cv2.waitKey(0)
        if key == ord('y'):
            #print('append image')
            imgpoints.append(corners2)
            filenames.append(fname)
        elif key == ord('n'):
            #print('discard image')
            continue
        else:
            #print('quit')
            sys.exit()

        cv2.destroyAllWindows()

np.save('./'+str(folder)+'/imgpoints.npy',imgpoints)
np.save('./'+str(folder)+'/filenames.npy',filenames)

cv2.destroyAllWindows()