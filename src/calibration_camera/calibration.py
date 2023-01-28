import numpy as np
import cv2 as cv
import glob
import os.path
#from undistort import undistort_image
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*10,3), np.float32)
objp[:,:2] = np.mgrid[0:10,0:7].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('calib_data/*.png')
img_counter = 0

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (10,7), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (10,7), corners2, ret)
        cv.imshow('img', img)
        img_name = "calib{}.png".format(img_counter)
        cv.imwrite(img_name, img)
        img_counter += 1

        
        cv.waitKey(500)
        # returns the camera matrix, distortion coefficients, rotation and translation vectors etc.
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        img = cv.imread('test_data/test_data7.png')
        h,  w = img.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        # print('distortion coefficents',dist)
        # Undistort
        dst = cv.undistort(img, mtx, dist, None, newcameramtx)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        cv.imwrite('calibresult.png', dst)

        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
            mean_error += error
        print( "total error: {}".format(mean_error/len(objpoints)) )

        rvecs=np.asarray(rvecs)#reshape from tuple to array
        tvecs=np.asarray(tvecs)

        mtx_txt = np.savetxt('camera_matrix.txt', mtx, fmt='%.2f')   
        dist_txt = np.savetxt('distortion_coefficients.txt', dist, fmt='%.2f')   
        rvecs_txt = np.savetxt("rotation_vector.txt", rvecs.reshape((3,-1)), fmt="%s")
        tvecs_txt = np.savetxt("translation_vector.txt", rvecs.reshape((3,-1)), fmt="%s")
cv.destroyAllWindows()
