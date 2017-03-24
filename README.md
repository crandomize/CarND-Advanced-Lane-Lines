# Advanced Lane Lines 

This document describes the implamentation of the project.
---

**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[imagecalibration]: ./writeupimages/calibrations.png
[imagedecomposed]: ./writeupimages/image_decomposed.png
[imagedechannels]: ./writeupimages/image_channels.png
[imagethresholds]:  ./writeupimages/image_thresholds.png
[perspective_transformation]: ./writeupimages/perspective_transformation.png
[full_transformation]: ./writeupimages/full_transformation.png

[radiuscurvature]: ./writeupimages/radiuscurvature.png


[lineid1]: ./writeupimages/line_id1.png
[lineid2]: ./writeupimages/line_id2.png
[lineid3]: ./writeupimages/line_id3.png
[lineid4]: ./writeupimages/line_id4.png

[snap1]: ./writeupimages/snap1.png
[snap2]: ./writeupimages/snap2.png


## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

This project contains the following main files:
 * camera_calibration.py  
 * image_pipeline.py
 * linedetection.py
 * project_notebook.ipynb

The notebook contains step by step code to generate the supported images as well as videos.  It may use the code in the py files or show it in the notebook directly without using the functions created in those py files as a way to clearly show some of the procedures used directly on the notebook.

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.   

You're reading it!

### Camera Calibration

#### 1. Camera matrix and distortion coefficients. Example of a distortion corrected calibration image.

The code for this step is contained in the cells 2-6 of the IPython notebook located in "./project_notebook.ipynb" (or in file `camera_calibration.py`).  

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image. 

Function `getAllCalibrationPoint` in camera_calibration.py file prepares and fetches all possible points detected.
Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to some of the test images using the `cv2.undistort()` function and obtained this result: 

![calibration][imagecalibration]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.
To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one:
![Image decomposed][imagedecomposed]

#### 2. Color Transform, gradients and binary image.

An image could be decomposed in different channels, like in below image comparison.

I used a combination of color and gradient thresholds to generate a binary image (thresholding steps at lines 12 through 46 in function thresholdColorGradient from `image_pipeline.py`).  

Below is an image of the different color channels

![Image Channels][imagedechannels]

From the channel S (last line of images) we apply a Sobel derivative in x direction and from channel V a threshold color channel.  Both are then combined.  The individual contribution is shown in below image.

In green: Sobel X
In blue: Color channel thresholding

![Image thresholds][imagethresholds]

#### 3. Perspective Transformation

The code for my perspective transform includes a function called `perspectiveTransformation()`, which appears in lines 49 through 66 in the file `image_pipeline.py`  (additionally you can see it in the IPython notebook).  

![Perspective transformation][perspective_transformation]

This above picture is possible to see the transformation points from the source image and the position in the result image

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 254, 683    | 270, 720       | 
| 600, 445     | 270, 0      |
| 680, 445    | 1000, 0      |
| 1060, 683     | 1000, 720        |

These points were choosen based on knowledge about this road and manually tuned to unward the first image.

Applying above color transformation and perspective transformation steps we obtain following images from the test images.

![Image transformation][full_transformation]


#### 4. Lane identification and polynomial fit

First we detect where the lines may start, based on a histogram of the lower part of the image.

So from original image:

![line id 1][lineid1]

We do the histogram obtaining the following:

![line id 1][lineid2]

These peaks are used as a starting point where to look for the lines.
We then start iterating and adjusting rectangular shapes to allow filtering the non interesting points.

![line id 1][lineid3]

In following picture we can see the result on test image.  Note in one of the image that the lanes found are not correct.  Although is clear that it can be improved , the system is more robust in the video analysis as previous images are also used to delimit the location of search points.

![line id 1][lineid4]


#### 5. Radious of curvature and lane position.

This is calculated in lines 145 through 165 in my code in `linedetection.py`

![radius][radiuscurvature]

As can be seen on above picture it's showing the curvature and lane position offset, both of them in meters.

The calculation is using following formula

```
            f(y) = Ax^2 + Bx + C

                    (1 + (2Ay + B)^2)^(3/2)
            Rcurve= -----------------------
                            |2A|
```

This is done after a conversion from pixels to meters space, based on the pixel/meter ratio, which can be easily calculated taking into account average line separation.

#### 6. Image Line areas identify scrrenshots

I implemented this step in lines 177 through 270 in my code in `linedetection.py` in the function `processImage()`.  Here is an example of my result on a set of tests image:

![alt text][snap1]

![alt text][snap2]

Note how the color of the printed image changes from green (good level of certainty) to more red when the uncertainty increases.

---

### Pipeline (video)

#### 1. Video.

Here's a [link to my video result](./test_project_video.mp4)

See it in [youtube](https://www.youtube.com/watch?v=ScBIXVCP5R8)

---

### Discussion

The parameters have been tuned for this example and therefore it won't be very robust in different roads.  Additional tunning and more tests should be implemented, perhaps parallels line detectors for different type of roads shoulde be used and do a final average from different detectors.



