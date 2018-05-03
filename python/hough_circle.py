import numpy as np
import argparse
import cv2
import imutils

class houghCircle:
    def __init__(self):
        pass

    def construct(self, image):

        image = imutils.resize(image, width=750)
        # cv2.imshow("image", image)
        # cv2.waitKey(0)
        output = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # detect circles in the image
        # circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 20)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 100,
                                   param1=100, param2=28, minRadius=0, maxRadius=30)

        # ensure at least some circles were found
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")

            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

            # show the output image
            # cv2.imshow("output", np.hstack([image, output]))
            # cv2.waitKey(0)

        return circles

