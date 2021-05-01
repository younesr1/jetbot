import numpy as np
import cv2 as cv
import glob
rows = 7
cols = 10
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((rows*cols, 3), np.float32)
objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.
images = glob.glob('images/*.jpg')
for fname in images:
    #print("##############################################################")
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (cols, rows), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (cols, cols), (-1, -1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        #cv.drawChessboardCorners(img, (rows, cols), corners2, ret)
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        # undistort
        dst = cv.undistort(img, mtx, dist, None, None)
        comp = np.concatenate((img, dst), axis=1)
        np.savetxt("intrinsics.txt", mtx)
        np.savetxt("distortions.txt", dist)
        cv.imshow("Comparison", comp)
        cv.waitKey(0)
    else:
        print("Failed")
cv.destroyAllWindows()