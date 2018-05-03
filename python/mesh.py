from shapeDetector import shapeDetector
from hough_circle import houghCircle
import cv2
import math

class meshingAlg:

    def __init__(self):
        pass

    def mesh(self, image):
        output = image.copy()

        # initialize Algorithm to detect Shapes
        main = shapeDetector()
        contours, shapes, ratio = main.construct(image)

        # initialize Algorithm to detect Circles
        hough = houghCircle()
        circles = hough.construct(image)

        epsilon = 15  # the threshold in pixels allowed

        markers = []

        # ensure at least some circles were found
        if circles is not None:
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                    i = 0
                    #print(x, y, r)
                    for c in contours:

                        # compute the center of the contour, then detect the name of the
                        # shape using only the contour
                        M = cv2.moments(c)
                        cX = int((M["m10"] / M["m00"]) * ratio)
                        cY = int((M["m01"] / M["m00"]) * ratio)

                        distX = x - cX
                        distY = y - cY

                        if math.sqrt(pow(distX, 2) + pow(distY, 2)) < epsilon:
                            shape = shapes[i]

                            area = cv2.contourArea(c)
                            #print(area)
                            if area > 500 and area < 1000:

                                # multiply the contour (x, y)-coordinates by the resize ratio,
                                # then draw the contours and the name of the shape on the image
                                c = c.astype("float")
                                c *= ratio
                                c = c.astype("int")
                                cv2.drawContours(image, [c], -1, (0, 255, 0), 1)
                                cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                                            0.5, (255, 255, 255), 1)

                                # show the output image
                                # resized = imutils.resize(image, width=750)
                                # cv2.imshow("Image", resized)
                                # cv2.waitKey(0)

                                # # draw the circle in the output image, then draw a rectangle
                                # # corresponding to the center of the circle
                                cv2.circle(image, (x, y), r, (0, 255, 0), 4)
                                cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

                                markers.append([x, y])

                                # cv2.imshow("output", np.hstack([image, output]))
                                # cv2.waitKey(0)
                        i = i + 1

        # # show the output image
        # cv2.imshow("output", np.hstack([image, output]))
        # cv2.waitKey(0)
        return image, markers
