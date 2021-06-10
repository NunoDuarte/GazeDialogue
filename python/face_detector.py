import cv2
import os


class FaceDetector:

    def __init__(self):
        self.subjects = ["", "iCub"]
        self.eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')
        self.face_cascade = cv2.CascadeClassifier("cascade-icub-60v60.xml")

    def detecting(self, frame):

        face = []
        # if you want to see the potential face found
        face_crop = []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (31, 31), 0)
        thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_TOZERO)[1]
        potential_face = self.face_cascade.detectMultiScale(
            thresh,
            scaleFactor=1.2,
            minNeighbors=5,
            minSize=(40, 40),
            maxSize=(200, 200),
            flags=cv2.CASCADE_FIND_BIGGEST_OBJECT
        )

        # you should only find one face
        # Draw a rectangle around the potential face
        for (x, y, w, h) in potential_face:
            num_eyes = 0

            roi_gray = gray[y:y + h, x:x + w]
            roi_color = frame[y:y + h, x:x + w]

            # find the eyes of the potential face
            eyes = self.eye_cascade.detectMultiScale(roi_gray)
            for (ex, ey, ew, eh) in eyes:
                if len(eyes) == 2:
                    cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 2)
                    num_eyes = 2
            if num_eyes == 2:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                face.append([x, y, w, h])
                face_crop.append(gray[y:y + w, x:x + h])
            # if not - ignore potential face

        return face

    def prepare_training_data(self, data_folder_path):

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

                non, non1, face = self.detecting(image, 0, self.face_cascade)

                if face is not None and len(face) is not 0:
                    faces.append(face[0])
                    labels.append(label)

                    cv2.destroyAllWindows()

        return faces, labels

    def predict(self, frame, face_recognizer, faces, facesTrain):

        if faces is not None and not []:
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



