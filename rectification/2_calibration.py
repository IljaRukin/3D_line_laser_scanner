import numpy as np
import cv2
import glob

#load data
folder = 'images'
imgpoints = np.load('./'+str(folder)+'/imgpoints.npy')
filenames = np.load('./'+str(folder)+'/filenames.npy')

n_corners = (9,6)
objp = np.zeros((n_corners[0]*n_corners[1],3), np.float32)
objp[:,:2] = np.mgrid[0:n_corners[0],0:n_corners[1]].T.reshape(-1,2)
objpoints = [objp]*len(filenames)

#example image
images = glob.glob('./'+str(folder)+'/*.jpg')
distorted = cv2.imread(images[0])
h,  w = distorted.shape[:2]

#calibrate
#ret = average mean square error in pixel
#mtx = camera matrix
#dist = distorsion
#rvecs/tvecs = rotation/translation vectors
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (w,h),(w,h),None)

#adjust camera matrix
#newcameramtx = adjusted camera matrix for cropped image
#roi = bounds for croping image
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

#undistort
#undistorted = cv2.undistort(distorted, mtx, dist, None, newcameramtx)

#alternative undistort
mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),cv2.CV_32FC1)
np.save('./'+str(folder)+'/mapx',mapx)
np.save('./'+str(folder)+'/mapy',mapy)
undistorted = cv2.remap(distorted,mapx,mapy,cv2.INTER_CUBIC)

# crop the image
#x,y,w,h = roi
#undistorted = undistorted[y:y+h, x:x+w]

scaler = 0.3
shape = np.array(distorted.shape)
dim = (np.array([shape[1],shape[0]])*scaler).astype(int)
distorted = cv2.resize(distorted,dim)
cv2.imshow( 'original image', distorted) #display image
cv2.moveWindow('original image', 50, 50) #window position
undistorted = cv2.resize(undistorted,dim)
cv2.imshow( 'undistorted image', undistorted) #display image
cv2.moveWindow('undistorted image', 550, 50) #window position
#cv2.cvtColor(distorted,cv2.COLOR_BGR2RGB)
#cv2.cvtColor(undistorted,cv2.COLOR_BGR2RGB)

key = cv2.waitKey(0)
cv2.destroyAllWindows()