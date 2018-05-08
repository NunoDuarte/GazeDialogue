import math
from pylsl import StreamInfo, StreamOutlet

class gazeBehaviour:

    def __init__(self, outlet=None):
        pass

    def record(self, timestamp, markers, allBalls, faces, fixation, labels):
        epsilon = 30  # the threshold in pixels allowed

        for ball in allBalls:
            distX = fixation[0] - ball[0][0][0]
            distY = fixation[1] - ball[0][0][1]

            if math.sqrt(pow(distX, 2) + pow(distY, 2)) < epsilon:
                #           time        ball    x           y           color of ball
                mysample = [timestamp, 0.0, ball[0][0][0], ball[0][0][1], ball[1]]
                return mysample

            for marker in markers:

                cX = marker[0]
                cY = marker[1]

                distX = fixation[0] - cX
                distY = fixation[1] - cY

                if math.sqrt(pow(distX, 2) + pow(distY, 2)) < epsilon:
                    mysample = [timestamp, 1.0, marker[0], marker[1]]
                    return mysample

            i = 0
            for face in faces:

                cX = face[0]
                cY = face[1]
                cW = face[0] + face[2]
                cH = face[1] + face[3]

                if cX - 30 < fixation[0] < cW + 30 and cY - 30 < fixation[1] < cH + 30:
                    mysample = [timestamp, 1+i, face[0], face[1]]
                    return mysample

                i = i+1
            mysample = [timestamp, 4.0, 0.0, 0.0]
        return mysample

