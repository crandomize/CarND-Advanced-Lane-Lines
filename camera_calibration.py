'''
 Camera calibration and correction functions

'''
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


def getAllCalibrationPoints(files, cx, cy):
    '''
    Finds all calibration points from the calibration images
    Args:
        files: array containing all files from calibration images to process
        cx : Number of square corners in horizontal line
        cy : Number of square corners in vertical line
    Returns:
        imgpoints : 2D points in image plane
        objpoints: 3D points in real world space
    '''
    
    # Prepare the arrays that will hold the points
    
    objpoints = [] # 3D points in real world space
    imgpoints = [] # 2D points in image plane
    objp = np.zeros((cx*cy, 3), np.float32)
    objp[:,:2] = np.mgrid[0:cx, 0:cy].T.reshape(-1, 2)

    
    for fname in files:
        img = mpimg.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        # Find the chessboard corners
        ret, corners= cv2.findChessboardCorners(gray, (cx, cy), None)

        # If corners are found, add object points, image points
        if ret == True:
            imgpoints.append(corners)
            objpoints.append(objp)
            
    return imgpoints, objpoints



def getCheesboardCorners(image, num_x, num_y):
    '''
    Find Cheesboard corners from image file
    Args:
        num_x : Number of square corners in horizontal line
        num_y : Number of square corners in vertical line
        image: Image
    Returns:
        ret: True if corners found
        corners: Array of corners (2D)
    '''

    #img = mpimg.imread(imfile)
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    # Find the chessboard corners
    ret, corners= cv2.findChessboardCorners(gray, (num_x, num_y), None)
    
    return ret, corners

def getImageCheesboardCorners(imfile, num_x, num_y):
    '''
    Return image of the corners drawn on top of the image
    '''
    img = mpimg.imread(imfile)

    ret, corners = getCheesboardCorners(img, num_x, num_y)
    img = cv2.drawChessboardCorners(img, (num_x, num_y), corners, ret)
    
    return img


