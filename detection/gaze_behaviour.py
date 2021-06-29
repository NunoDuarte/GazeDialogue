import math
import cv2


class GazeBehaviour:

    def __init__(self, outlet=None):
        pass

    def record(self, timestamp, allBalls, face, fixation, labels, width, height):
        epsilon = 80  # the threshold in pixels allowed

        for ball in allBalls:
            if len(ball[0]) == 1:
                distX = fixation[0] - ball[0][0][0]
                distY = fixation[1] - ball[0][0][1]

                if math.sqrt(pow(distX, 2) + pow(distY, 2)) < epsilon:
                    #           time        color of ball    x           y
                    mysample = [timestamp, ball[1], ball[0][0][0], ball[0][0][1], ]
                    return mysample

            if face is not None:
                cX = face[0]*width
                cY = face[1]*height
                cW = face[0]*width + face[2]*width
                cH = face[1]*height + face[3]*height
                print(cX, cY, cW, cH)

                if cX - epsilon < fixation[0] < cW + epsilon and cY - epsilon < fixation[1] < cH + epsilon:
                    mysample = [timestamp, 7, face[0], face[1]]
                    return mysample

            mysample = []

        return mysample

    def push(self, frame, sample, ball, face, width, height, lsl):
        pos_x = sample[0][1]
        pos_y = sample[0][2]

        # print(int(float(pos_x)*width))
        # print(int(height - int(float(pos_y)*height)))
        cv2.circle(frame, (int(float(pos_x) * width), int(height - int(float(pos_y) * height))), 10, (0, 255, 1),
                   thickness=5, lineType=8, shift=0)  # draw circle
        fixation = [(int(float(pos_x) * width)), int(height - int(float(pos_y) * height))]

        # check the gaze behaviour
        if len(ball) is not 0:
            mysample = self.record(sample[0][0], ball, face, fixation, [], width, height)
            if len(mysample) is not 0:
                print(mysample)
                # lsl.outlet.push_sample(mysample)

