import math
from pylsl import StreamInfo, StreamOutlet


class gazeBehaviour:

    def __init__(self, outlet=None):
        pass

    def record(self, timestamp, allBalls, faces, fixation, labels):
        epsilon = 8  # the threshold in pixels allowed

        for ball in allBalls:
            if len(ball[0]) == 1:
                distX = fixation[0] - ball[0][0][0]
                distY = fixation[1] - ball[0][0][1]

                if math.sqrt(pow(distX, 2) + pow(distY, 2)) < epsilon:
                    #           time        color of ball    x           y
                    mysample = [timestamp, ball[1], ball[0][0][0], ball[0][0][1], ]
                    return mysample

                i = 0
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

