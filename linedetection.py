import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import glob

import camera_calibration as cc
import image_pipeline as ip

class Line():
    def __init__(self):
        # was the line detected in the last iteration?
        self.detected = False  
        # x values of the last n fits of the line
        self.recent_xfitted = [] 
        #average x values of the fitted line over the last n iterations
        self.bestx = None     
        #polynomial coefficients averaged over the last n iterations
        self.best_fit = None  
        #polynomial coefficients for the most recent fit
        self.current_fit = [np.array([False])]  
        
        #previous fits to do an average
        self.previous_fits = []
        
        #radius of curvature of the line in some units
        self.radius_of_curvature = None 
        #distance in meters of vehicle center from the line
        self.line_base_pos = None 
        #difference in fit coefficients between last and new fits
        self.diffs = np.array([0,0,0], dtype='float') 
        #x values for detected line pixels
        self.allx = None  
        #y values for detected line pixels
        self.ally = None
        
    def addFit(self, fit):
        
        num = len(self.previous_fits)
        if num > 6:
            self.previous_fits.pop(0)
        self.previous_fits.append(fit)
        self.current_fit = fit
        
    def getAverageFit(self, lastn = 5):
        return np.average(self.previous_fits[-lastn:], 0)
        
        
class LineDetection():
    def __init__(self):
        self.detected = False
        
        # camera calibration
        self.mtx    = None
        self.dist   = None
        
        # Perspective transformation matrix
        self.M      = None
        self.Minv   = None
        
        # Lines
        self.lineRight = Line()
        self.lineLeft = Line()
        
        # Previous Parallel line Error
        self.pe = None
        
        # Define number of meters per pixel
        self.ym_per_pix = 30/720  # meters per pixel in y dimension
        self.xm_per_pix = 3.7/700 # meters per pixel in x dimension
        
    def calibrateFromFiles(self,imagefiles, cx, cy):
        '''
        Calibrate the camera from the files
        '''
        
        # Calibrate image
        # Prepare the arrays that will hold the points
        
        imgpoints, objpoints =  cc.getAllCalibrationPoints(imagefiles, cx, cy)
        img_size = mpimg.imread(imagefiles[0]).shape
        img_size = (img_size[1], img_size[0])
        
        # Do camera calibration given object points and image points
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size,None,None)
    
        self.mtx = mtx
        self.dist = dist
        
    
        
    def undistortedImage(self, image): 
        '''
        Return undistorted image using already calibrated parameters
        If not camera calibrated return image
        '''
        if self.mtx != None:
            return cv2.undistort(image, self.mtx, self.dist, None, self.mtx)
        else:
            return image
        
        
    def setPerspectiveTransformationMatrix(self, pointsSource, pointsDest):
        '''
        Setup matrix for perspective Transformation and its inverse
        Parameters:
            pointsSource = np.float32([(254, 683), (600, 445), (680, 445), (1060, 683)])
            pointsDest = np.float32([(270, 720), (270, 0), (1000, 0), (1000, 720)])
        '''
        
        self.M = cv2.getPerspectiveTransform(pointsSource, pointsDest)        
        self.Minv = cv2.getPerspectiveTransform(pointsDest, pointsSource)

    def prepareImageLines(self, image):
        '''
        Transform image to make it possible to detect the lines (perspective Transformation, etc)
        '''
           
        dst = ip.perspectiveTransformation(image, self.mtx, self.dist, self.M, True)
    
        return dst
    
    
    def getXPos(self, fit, y):
        '''
        Get the X position from Y and the fit 
        '''
        return fit[0]*y**2 + fit[1]*y + fit[2]
        
    def paralelError(self, left_fit, right_fit):
        '''
        Defines standard error of parallels, the bigger the less parallel they are
        '''
        num = 5
        ploty = np.linspace(700, 10, num)
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]    
    
        adiff = right_fitx - left_fitx
        adiffavg = np.average(adiff)
        total = np.square(adiff - adiffavg)  
        parallelerror = np.sqrt(np.sum(total))/num
        return round(parallelerror,1)
    
    def curvature(self, fit, y):
        '''
        Measure the curvature based on the fit  
            f(y) = Ax^2 + Bx + C
                    (1 + (2Ay + B)^2)^(3/2)
            Rcurve= -----------------------
                            |2A|
                            
        Parameters:
            fit: array[ A, B, C]
            y: point where to measure curvature
        Returns:
            curvature
        '''
        
        curvature = ((1 + (2*fit[0]*y + fit[1])**2)**1.5) / np.absolute(2*fit[0])
        
        return curvature
        
        
    def fitPixels2Meters(self, fit, pixelsheight):
        '''
        Change fit parameters from pixel space to meter space
        '''
        
        ploty = np.linspace(0, pixelsheight - 1, pixelsheight)
        plotx = fit[0]*ploty**2 + fit[1]*ploty + fit[2]
        
        fit_cr = np.polyfit(ploty*self.ym_per_pix, plotx*self.xm_per_pix, 2)
        
        return fit_cr
        
    def processImage(self, image):
        
        warped = self.prepareImageLines(image)
        
        if (self.pe is not None and self.pe < 50):
            left_fit, right_fit = ip.findLinesFromFit(image, self.lineLeft.current_fit, self.lineRight.current_fit)
        else:
            left_fit, right_fit = ip.findLines(warped)
        
        #self.lineLeft.current_fit = left_fit
        #self.lineRight.current_fit = right_fit
        
        pe = self.paralelError(left_fit, right_fit)
        
        if pe < 30:
            self.lineLeft.addFit(left_fit)
            self.lineRight.addFit(right_fit)
            
        left_fit = self.lineLeft.getAverageFit(6)
        right_fit = self.lineRight.getAverageFit(6)
        
        
        # Get the width of the Line and the offset in meters
        
        leftLinePos = self.getXPos(left_fit, image.shape[0])
        rightLinePos = self.getXPos(right_fit, image.shape[0])
        widthline = (rightLinePos - leftLinePos)
        offset = np.abs(image.shape[1]/2 - (leftLinePos + widthline/2))
        
        widthlineM = widthline*self.xm_per_pix
        offsetM = offset*self.xm_per_pix
        
        widthError = np.abs(3.75 - widthlineM)
        offsetError = offsetM
        
        totalError = widthError*20 + offsetError*5 + pe/10
        
        ploty = np.linspace(0, image.shape[0]-1, image.shape[0] )
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        
        
        # Create an image to draw the lines on
        warp_zero = np.zeros_like(warped).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        
        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))
        
        # Draw the lane onto the warped blank image
        rcolor = np.min([255, totalError*10])
        gcolor = 255 - rcolor
        cv2.fillPoly(color_warp, np.int_([pts]), (rcolor, gcolor, 0))
        #cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))
        
        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, self.Minv, (image.shape[1], image.shape[0])) 
        # Combine the result with the original image
        result = cv2.addWeighted(image, 1, newwarp, 0.3, 0)
        
        curvature_left  = self.curvature( self.fitPixels2Meters(left_fit,  image.shape[0]), image.shape[0]*self.ym_per_pix)
        curvature_right = self.curvature( self.fitPixels2Meters(right_fit, image.shape[0]), image.shape[0]*self.ym_per_pix)
        
        self.lineLeft.radius_of_curvature = curvature_left
        self.lineRight.radius_of_curvature = curvature_right
        
        averageradius = (curvature_left + curvature_right)//2
        cv2.putText(result,' '.join(["Error Level:" , str(round(totalError,1))]), (40,50), cv2.FONT_HERSHEY_SIMPLEX, 1, 255)
        cv2.putText(result,' '.join(["Curvature:" , str(averageradius)]), (40,80), cv2.FONT_HERSHEY_SIMPLEX, 1, 255)
        #cv2.putText(result,' '.join(["CR:" , str(round(curvature_right,1))]), (40,110), cv2.FONT_HERSHEY_SIMPLEX, 1, 255)
        

 
        cv2.putText(result,' '.join(["Width Line:" , str(round(widthlineM,2))]), (40,110), cv2.FONT_HERSHEY_SIMPLEX, 1, 255)
        cv2.putText(result,' '.join(["Offset:" , str(round(offsetM, 2))]), (40,140), cv2.FONT_HERSHEY_SIMPLEX, 1, 255)       
        
        
        
        # Add warped image
        x_offset=960
        y_offset=20
        warped = cv2.resize(warped, (300,150))*255
        
        result[y_offset:y_offset+warped.shape[0], x_offset:x_offset+warped.shape[1]] = np.dstack((warped, warped, warped))
        
        x_offset=650
        y_offset=20
        result[y_offset:y_offset+warped.shape[0], x_offset:x_offset+warped.shape[1]] = cv2.resize(color_warp[:,:,[1,0,2]], (300,150))*255



        return result
    
    
    