import numpy as np
import cv2

class redBall:

    def __init__(self):
        pass

    def tracking(self, frame, pts, args):
        # blur the frame, and convert it to the HSV
        # color space
        output1 = frame.copy()

        ball = []

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # because hue wraps up and to extract as many "red objects" as possible,
        # I define lower and upper boundaries for brighter and for darker red shades
        bright_red_lower_bounds = (0, 100, 100)
        bright_red_upper_bounds = (10, 255, 255)
        bright_red_mask = cv2.inRange(hsv, bright_red_lower_bounds, bright_red_upper_bounds)

        dark_red_lower_bounds = (160, 100, 100)
        dark_red_upper_bounds = (179, 255, 255)
        dark_red_mask = cv2.inRange(hsv, dark_red_lower_bounds, dark_red_upper_bounds)

        # after masking the red shades out, I add the two images
        weighted_mask = cv2.addWeighted(bright_red_mask, 1.0, dark_red_mask, 1.0, 0.0)

        # then the result is blurred
        blurred = cv2.GaussianBlur(weighted_mask, (9, 9), 3, 3)

        # construct a mask for the color "red", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        # mask = cv2.inRange(blurred, greenLower, greenUpper)
        mask = cv2.erode(blurred, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 1:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                #cv2.circle(frame, (int(x), int(y)), int(radius),
                #           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                ball.append([int(x), int(y)])

        # loop over the set of tracked points
        for i in range(1, len(pts)):
            # if either of the tracked points are None, ignore
            # them
            if pts[i - 1] is None or pts[i] is None:
                continue

            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
            cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

        # update the points queue
        pts.appendleft(center)

        # show the frame to our screen
        # cv2.imshow("output1", np.hstack([frame, output1]))
        return frame, pts, ball


