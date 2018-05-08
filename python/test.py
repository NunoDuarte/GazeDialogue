# import files
from mesh import meshingAlg
from find_nearest import find_nearest
from redBalltracking import redBall
from faceDetector import faceDetector
from gazeBehaviouroff import gazeBehaviour
# import necessary libraries
from collections import deque
import numpy as np
import cv2
import csv
import os
import argparse
import imutils
import logging as log

import numpy as np
import cv2 as cv
face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
# eye_cascade = cv.CascadeClassifier('haarcascade_eye.xml')
# image = cv.imread('icub_face.png')
# cv2.imshow('Raw', image)
#
# # if cv2.waitKey() == ord('q'):
# #     cv2.destroyAllWindows()
# image[np.where((image == [208, 225, 244]).all(axis = 2))] = [3, 7, 14]
# cv2.imwrite('output.png', image)
#
# # lower_red = np.array([69, 118, 247], dtype="uint16")
# # upper_red = np.array([92, 153, 255], dtype="uint16")
# # black_mask = cv2.inRange(image, lower_red, upper_red)
# # newImg = image.copy()
# # mat = np.where((black_mask == [255]))
# # print(mat[0])
# # print(mat[1])
# # for submat0 in mat[0]:
# #     for submat1 in mat[1]:
# #         newImg[submat0][submat1] == [0, 0, 0]
#
# # newImg[np.where((black_mask == [255]))] == [0, 0, 0]
# # file = open('text.txt', 'w')
# # i = 0
# # for mask in newImg:
# #     for submask in mask:
# #         file.write(str(submask) + str(i))
# #         file.write('\n')
# #         i = i + 1
# cv2.imwrite('output1.png', newImg)
# # image[np.where((image == [255, 255, 255]).all(axis=2))] = [0, 0, 0]  # it works
# # black_mask[np.where(black_mask == [10])] = [0]
# # black_mask[np.where((black_mask == [0, 0, 0]).all(axis=2))] = [0, 255, 255]  # it doesn't work
# cv2.imshow('Test', image)
# cv2.imshow('Test2', black_mask)
# cv2.imwrite('output.png', black_mask)
#
# # print(type(img))
# # img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# # new = [[[0, 0, 255 % j] for j in i] for i in img_gray]
# # dt = np.dtype('f8')
# # new = np.array(new, dtype=dt)
# # new1 = new/255
#
# # cv2.imwrite('img.jpg', new)
# cv2.imshow('Test', newImg)

if cv2.waitKey() == ord('q'):
    cv2.destroyAllWindows()

img = cv.imread('output1.png')

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

faces = face_cascade.detectMultiScale(gray, 1.3, 5)
for (x,y,w,h) in faces:
    cv.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
    roi_gray = gray[y:y+h, x:x+w]
    roi_color = img[y:y+h, x:x+w]
    # # eyes = eye_cascade.detectMultiScale(roi_gray)
    # for (ex,ey,ew,eh) in eyes:
    #     cv.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
cv.imshow('img',img)
cv.waitKey(0)
cv.destroyAllWindows()