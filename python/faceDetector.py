import cv2
import logging as log
import datetime as dt
import os

class faceDetector:

    def __init__(self):
        self.subjects = ["", "iCub"]

    def detecting(self, frame, anterior, faceCascade):

        faces = []
        faceTrain = []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (31, 31), 0)
        thresh = cv2.threshold(blurred, 127, 255,cv2.THRESH_TOZERO)[1]
        facesDetect = faceCascade.detectMultiScale(
            thresh,
            scaleFactor=1.3,
            minNeighbors=10,
            minSize=(50, 50),
	        maxSize=(100, 100)
        )

        # Draw a rectangle around the faces
        for (x, y, w, h) in facesDetect:
            #print(x,y,w,h)
            if x > 50 and x < 350 and y < 150:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                faces.append([x, y, w, h])
                faceTrain.append(gray[y:y+w, x:x+h])
                break

        if anterior != len(facesDetect):
            anterior = len(facesDetect)
            log.info("faces: "+str(len(facesDetect))+" at "+str(dt.datetime.now()))

        return anterior, faces, faceTrain

    def prepare_training_data(self, data_folder_path, faceCascade):

        dirs = os.listdir(data_folder_path)

        faces = []
        labels = []

        for dir_name in dirs:
            if not dir_name.startswith("s"):
                continue

            label = int(dir_name.replace("s", ""))

            subject_dir_path = data_folder_path + "/" + dir_name

            subject_images_names = os.listdir(subject_dir_path)

            for image_name in subject_images_names:
                if image_name.startswith("."):
                    continue

                image_path = subject_dir_path + "/" + image_name
                image = cv2.imread(image_path)

                cv2.imshow("Training on image...", image)
                cv2.waitKey(100)

                non, non1, face = self.detecting(image, 0, faceCascade)

                if face is not None and len(face) is not 0:
                    faces.append(face[0])
                    labels.append(label)

                    cv2.destroyAllWindows()

        return faces, labels

    def predict(self, frame, face_recognizer, faces, facesTrain):

        if faces is not None:
            labels = []
            i = 0
            for face in facesTrain:

                label = face_recognizer.predict(face)
                #print(faces[i])
                label_text = self.subjects[label[0]]

                cv2.putText(frame, label_text, (faces[i][0], faces[i][1]-5), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0), 2)
                labels.append(label_text)
                i = i+1

            return labels



