import cv2 as cv
import numpy as np
import imutils as mul


vid = cv.VideoCapture(0)
  
while(True):
      
    # Capture the video frame
    # by frame
    ret, frame = vid.read()

    #reading image, resizing
    img = vid
    img = cv.resize(img, (800,450))

    #change to hsv color
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    #make range lower and upper to do masking
    lower=np.array([40, 0, 0])
    upper=np.array([90, 255, 255])

    #masking
    mask_mag=cv.inRange(hsv,lower,upper)
    res = cv.bitwise_and(img,img,mask=mask_mag)

    #change to gray color, blurring, and threshold to do countour
    gray = cv.cvtColor(res, cv.COLOR_BGR2GRAY)
    blurred = cv.GaussianBlur(gray, (5, 5), 0)
    thresh = cv.threshold(blurred, 100, 255, cv.THRESH_BINARY)[1]

    #find contour
    cnts = cv.findContours(thresh.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = mul.grab_contours(cnts)


    for c in cnts:
        # compute the center of the contour
        M = cv.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        # draw the contour and center of the shape on the image
        cv.drawContours(img, [c], -1, (0, 0, 0), 2)
        cv.circle(img, (cX, cY), 5, (0, 0, 255), -1)
        #putting text
        textField = "detected"
        cv.putText(img, textField, (cX - 55, cY - 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

    #make the result as a .jpg file and show

    cv.waitKey(0)
    cv.destroyAllWindows()