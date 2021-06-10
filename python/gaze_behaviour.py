import math
import cv2


class GazeBehaviour:

    def __init__(self, outlet=None):
        pass

    def record(self, timestamp, allBalls, faces, fixation):
        epsilon = 30  # the threshold in pixels allowed

        for ball in allBalls:
            if len(ball[0]) == 1:
                distX = fixation[0] - ball[0][0][0]
                distY = fixation[1] - ball[0][0][1]

                if math.sqrt(pow(distX, 2) + pow(distY, 2)) < epsilon:
                    #           time        color of ball    x           y
                    mysample = [timestamp, ball[1], ball[0][0][0], ball[0][0][1], ]
                    return mysample

                for face in faces:

                    cX = face[0]
                    cY = face[1]
                    cW = face[0] + face[2]
                    cH = face[1] + face[3]

                    if cX - 30 < fixation[0] < cW + 30 and cY - 30 < fixation[1] < cH + 30:
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
            mysample = self.record(sample[0][0], ball, face, fixation)
            if len(mysample) is not 0:
                # print(mysample)
                lsl.outlet.push_sample(mysample)

