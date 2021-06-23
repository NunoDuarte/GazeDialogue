import numpy as np
import cv2


class Ball:

    def __init__(self):
        self.ball_all = []

    def tracking(self, frame, vector):
        if vector[0]:
            self.green(frame)
        if vector[1]:
            self.red(frame)
        if vector[2]:
            self.blue(frame)
        if vector[3]:
            self.yellow(frame)
        if vector[4]:
            self.cyan(frame)

        return self.ball_all

    def cyan(self, frame):
        ball = []

        # hsv for cyan bounds
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        bright_cyan_lower_bounds = (75, 100, 100)
        bright_cyan_upper_bounds = (80, 255, 255)
        bright_cyan_mask = cv2.inRange(hsv, bright_cyan_lower_bounds, bright_cyan_upper_bounds)
        dark_cyan_lower_bounds = (90, 100, 100)
        dark_cyan_upper_bounds = (105, 255, 255)
        # mask
        dark_cyan_mask = cv2.inRange(hsv, dark_cyan_lower_bounds, dark_cyan_upper_bounds)
        weighted_mask = cv2.addWeighted(bright_cyan_mask, 1.0, dark_cyan_mask, 1.0, 0.0)
        # blur
        blurred = cv2.GaussianBlur(weighted_mask, (9, 9), 3, 3)
        # dilation and erosion to remove small blobs in the mask
        mask = cv2.erode(blurred, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contour
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        if len(cnts) > 0:
            # find largest contour. compute enclosing circle and centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 1:
                # draw circle and centroid on frame
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                ball.append([int(x), int(y)])

        if ball is not [] and len(ball) != 0:
            self.ball_all.append([ball, 5])

        return frame, ball

    def yellow(self, frame):
        ball = []

        # hsv for yellow bounds
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        bright_yellow_lower_bounds = (20, 100, 100)
        bright_yellow_upper_bounds = (30, 255, 255)
        bright_yellow_mask = cv2.inRange(hsv, bright_yellow_lower_bounds, bright_yellow_upper_bounds)
        dark_yellow_lower_bounds = (20, 100, 100)
        dark_yellow_upper_bounds = (30, 255, 255)
        # mask
        dark_yellow_mask = cv2.inRange(hsv, dark_yellow_lower_bounds, dark_yellow_upper_bounds)
        weighted_mask = cv2.addWeighted(bright_yellow_mask, 1.0, dark_yellow_mask, 1.0, 0.0)
        # blur
        blurred = cv2.GaussianBlur(weighted_mask, (9, 9), 3, 3)
        # dilation and erosion to remove small blobs in the mask
        mask = cv2.erode(blurred, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # find contour
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        if len(cnts) > 0:
            # find largest contour. compute enclosing circle and centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 1:
                # draw circle and centroid on frame
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                ball.append([int(x), int(y)])

        if ball is not [] and len(ball) != 0:
            self.ball_all.append([ball, 4])

        return frame, ball

    def red(self, frame):
        ball = []

        # hsv for red bounds
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        bright_red_lower_bounds = (0, 100, 100)
        bright_red_upper_bounds = (10, 255, 255)
        bright_red_mask = cv2.inRange(hsv, bright_red_lower_bounds, bright_red_upper_bounds)
        dark_red_lower_bounds = (160, 100, 100)
        dark_red_upper_bounds = (179, 255, 255)
        # mask
        dark_red_mask = cv2.inRange(hsv, dark_red_lower_bounds, dark_red_upper_bounds)
        weighted_mask = cv2.addWeighted(bright_red_mask, 1.0, dark_red_mask, 1.0, 0.0)
        # blur
        blurred = cv2.GaussianBlur(weighted_mask, (9, 9), 3, 3)
        # dilation and erosion to remove small blobs in the mask
        mask = cv2.erode(blurred, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # find contour
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        if len(cnts) > 0:
            # find largest contour. compute enclosing circle and centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 1:
                # draw circle and centroid on frame
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                ball.append([int(x), int(y)])

        if ball is not [] and len(ball) != 0:
            self.ball_all.append([ball, 4])  # 2

        return frame, ball

    def blue(self, frame):
        ball = []

        # hsv for blue bounds
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low_blue = np.array([94, 80, 2])
        high_blue = np.array([126, 255, 255])
        # mask
        blue_mask = cv2.inRange(hsv, low_blue, high_blue)
        # find contour
        cnts = cv2.findContours(blue_mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]

        center = None
        if len(cnts) > 0:
            # find largest contour. compute enclosing circle and centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 1:
                # draw circle and centroid on frame
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                ball.append([int(x), int(y)])

        if ball is not [] and len(ball) != 0:
            self.ball_all.append([ball, 1])  # 3

        return frame, ball

    def green(self, frame):
        ball = []

        # hsv for green bounds
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_low = np.array([45, 100, 50])
        green_high = np.array([75, 255, 255])
        # mask
        mask = cv2.inRange(hsv, green_low, green_high)
        # blur frame
        blurred = cv2.GaussianBlur(mask, (9, 9), 3, 3)
        # dilation and erosion to remove small blobs in the mask
        mask = cv2.erode(blurred, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # find contour
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]

        center = None
        if len(cnts) > 0:
            # find largest contour. compute enclosing circle and centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 1:
                # draw circle and centroid on frame
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                ball.append([int(x), int(y)])

        if ball is not [] and len(ball) != 0:
            self.ball_all.append([ball, 5])  # 1

        return frame, ball

