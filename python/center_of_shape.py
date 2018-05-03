# import the necessary packages
import argparse
import imutils
import cv2

class CenterShape:
    def __init__(self):
        pass

    def center(self, image):

        # load the image, convert it to grayscale, blur it slightly,
        # and threshold it
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
        # thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, \
        #                                  cv2.THRESH_BINARY_INV, 11, 2)
        ret3, thresh = cv2.threshold(blurred,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        # cv2.imshow('image1', image)
        # cv2.imshow('image2', gray)
        # cv2.imshow('image3', blurred)
        resized = imutils.resize(thresh, width=750)
        ratio = image.shape[0] / float(thresh.shape[0])

        # cv2.imshow('image4', resized)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()


        # find contours in the thresholded image
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]

        # loop over the contours
        for c in cnts:

            # compute the center of the contour
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # draw the contour and center of the shape on the image
                cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
                cv2.circle(image, (cX, cY), 7, (255, 255, 255), -1)
                cv2.putText(image, "center", (cX - 20, cY - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # show the image
                resized = imutils.resize(image, width=750)
                # cv2.imshow("Image", resized)
                # cv2.waitKey(0)

        return cnts, ratio
