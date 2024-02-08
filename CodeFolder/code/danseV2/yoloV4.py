import cv2
import numpy as np
from ultralytics import YOLO
import threading
import UdpComms as U
import queue


class PoseAnalyzer:
    def __init__(self):
        # Initialisation du modÃ¨le YOLO et du signal de dÃ©marrage
        self.model = None
        self.startSignal = threading.Event()
        self.udpCommunicator = U.UdpComms(
            udpIP="127.0.0.1", portTX=8000, portRX=8001, enableRX=True, suppressWarnings=True)
        self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        # File d'attente pour stocker les 5 derniÃ¨res frames
        self.frame_queue = queue.Queue(maxsize=5)
        self.initializeYolo()

    def initializeYolo(self):
        # Initialiser YOLO dans un thread sÃ©parÃ©
        self.initializeYoloThread()

    def initializeYoloThread(self):
        # Initialiser le modÃ¨le YOLO
        self.model = YOLO("yolov8n-pose.pt")
        self.startSignal.set()

    @staticmethod
    def calculateDistance(a, b):
        return np.sqrt(((a[0] - b[0]) ** 2) + ((a[1] - b[1]) ** 2))

    @staticmethod
    def calculateAngle(a, b, c):
        v1 = np.array([a[0] - b[0], a[1] - b[1]])
        v2 = np.array([c[0] - b[0], c[1] - b[1]])
        return np.degrees(np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))))

    @staticmethod
    def getSide(positions):
        leftSide = PoseAnalyzer.calculateDistance(
            positions["left_shoulder"], positions["left_hip"])
        rightSide = PoseAnalyzer.calculateDistance(
            positions["right_shoulder"], positions["right_hip"])
        return "gauche" if leftSide > rightSide else "droite" if leftSide < rightSide else "centre"

    @staticmethod
    def getHeadPosition(positions):
        # Calculer la position moyenne des yeux
        eye_avg_x = (positions["left_eye"][0] + positions["right_eye"][0]) / 2

        # Calculer la position moyenne des Ã©paules
        shoulder_avg_x = (positions["left_shoulder"]
                          [0] + positions["right_shoulder"][0]) / 2

        # Comparer la position des yeux et des Ã©paules pour dÃ©terminer la position de la tÃªte
        if eye_avg_x < shoulder_avg_x - 20:
            return "droite"
        elif eye_avg_x > shoulder_avg_x + 20:
            return "gauche"
        else:
            return "centre"

    @staticmethod
    def getHeadPositionY(positions):
        # Calculer la position moyenne des yeux sur l'axe Y
        eye_avg_y = (positions["left_eye"][1] + positions["right_eye"][1]) / 2

        # Calculer la position moyenne des Ã©paules sur l'axe Y
        shoulder_avg_y = (positions["left_shoulder"]
                          [1] + positions["right_shoulder"][1]) / 2

        # Calculer la position de la tÃªte sur l'axe Y
        head_y_position = (eye_avg_y + shoulder_avg_y) / 2

        return head_y_position

    @staticmethod
    def getWristPosition(positions):
        leftElbow = positions["left_elbow"]
        rightElbow = positions["right_elbow"]
        leftWrist = positions["left_wrist"]
        rightWrist = positions["right_wrist"]

        leftWristPosition = "down" if leftWrist[1] > leftElbow[1] else "up"
        rightWristPosition = "down" if rightWrist[1] > rightElbow[1] else "up"

        return leftWristPosition, rightWristPosition

    @staticmethod
    def getHeadInclination(positions):
        # Calculer la position moyenne des yeux
        eye_avg_x = (positions["left_eye"][0] + positions["right_eye"][0]) / 2
        eye_avg_y = (positions["left_eye"][1] + positions["right_eye"][1]) / 2

        # Utiliser la position du nez comme troisiÃ¨me point
        nose_x, nose_y = positions["nose"]

        # Calculer les vecteurs entre les yeux et le nez
        eye_to_nose_vector = np.array([nose_x - eye_avg_x, nose_y - eye_avg_y])
        horizontal_vector = np.array([1, 0])  # Vecteur horizontal

        # Calculer l'angle entre les vecteurs
        angle_radians = np.arccos(np.dot(eye_to_nose_vector, horizontal_vector) /
                                  (np.linalg.norm(eye_to_nose_vector) * np.linalg.norm(horizontal_vector)))
        angle_degrees = np.degrees(angle_radians)

        # Si l'angle est plus grand que 90 degrÃ©s, le tourner dans l'autre sens
        if nose_y > eye_avg_y:
            angle_degrees = -angle_degrees

        return angle_degrees

    def processFrame(self, frame):
        results = self.model(frame, conf=0.6)
        f = results[0].plot()
        keypoints = results[0].keypoints.xy.tolist()
        peoples = list()

        positions = {
            "nose": [0, 0],
            "left_eye": [0, 0],
            "right_eye": [0, 0],
            "left_ear": [0, 0],
            "right_ear": [0, 0],
            "left_shoulder": [0, 0],
            "right_shoulder": [0, 0],
            "left_elbow": [0, 0],
            "right_elbow": [0, 0],
            "left_wrist": [0, 0],
            "right_wrist": [0, 0],
            "left_hip": [0, 0],
            "right_hip": [0, 0],
            "left_knee": [0, 0],
            "right_knee": [0, 0],
            "left_ankle": [0, 0],
            "right_ankle": [0, 0]
        }
        
        # RÃ©cupÃ©ration des positions des points liÃ©s Ã  YOLO
        for person in keypoints:
            if not person:
                # Mise en place de position de base pour Ã©viter de faire planter le script si aucune personne
                continue

            positions = {
                "nose": person[0],
                "left_eye": person[1],
                "right_eye": person[2],
                "left_ear": person[3],
                "right_ear": person[4],
                "left_shoulder": person[5],
                "right_shoulder": person[6],
                "left_elbow": person[7],
                "right_elbow": person[8],
                "left_wrist": person[9],
                "right_wrist": person[10],
                "left_hip": person[11],
                "right_hip": person[12],
                "left_knee": person[13],
                "right_knee": person[14],
                "left_ankle": person[15],
                "right_ankle": person[16]
            }

            peoples.append(positions)

        if peoples:
            first_person = peoples[0]

            leftArmAngle = round(PoseAnalyzer.calculateAngle(
                positions["left_shoulder"], positions["left_elbow"], positions["left_wrist"]), 2)
            rightArmAngle = round(PoseAnalyzer.calculateAngle(
                positions["right_shoulder"], positions["right_elbow"], positions["right_wrist"]), 2)
            leftShoulderAngle = round(PoseAnalyzer.calculateAngle(
                positions["left_hip"], positions["left_shoulder"], positions["left_elbow"]), 2)
            rightShoulderAngle = round(PoseAnalyzer.calculateAngle(
                positions["right_hip"], positions["right_shoulder"], positions["right_elbow"]), 2)
            headPosition = PoseAnalyzer.getHeadPosition(positions)
            head_y_position = PoseAnalyzer.getHeadPositionY(positions)
            wristDistance = PoseAnalyzer.calculateDistance(positions["left_wrist"], positions["right_wrist"])

            leftWristPosition, rightWristPosition = self.getWristPosition(
                first_person)

            movement_data = {
                "leftArmAngle": leftArmAngle,
                "rightArmAngle": rightArmAngle,
                "leftShoulderAngle": leftShoulderAngle,
                "rightShoulderAngle": rightShoulderAngle,
                "leftWristPosition": leftWristPosition,
                "rightWristPosition": rightWristPosition,
                "headPosition": headPosition,
                "headPositionY": head_y_position,
                "wristDistance" : wristDistance,
            }

            self.drawAngles(f, positions, leftArmAngle, rightArmAngle,
                            leftShoulderAngle, rightShoulderAngle, headPosition, leftWristPosition, rightWristPosition)

            # self.udpCommunicator.SendData(str(movement_data))
             # Ajouter les donnÃ©es Ã  la file d'attente
            self.frame_queue.put(movement_data)
            # Si la file d'attente dÃ©passe 5 Ã©lÃ©ments, retirez le premier Ã©lÃ©ment
            if self.frame_queue.full():
                self.frame_queue.get()
                
            # Calculer la moyenne des donnÃ©es dans la file d'attente
            avg_movement_data = self.calculateAverage()
            # Envoyer la moyenne Ã  un autre endroit
            self.sendAverage(avg_movement_data)

        return f

    @staticmethod
    def drawAngles(frame, positions, leftArmAngle, rightArmAngle, leftShoulderAngle, rightShoulderAngle, headPosition, leftWristPosition, rightWristPosition):
        cv2.putText(frame, str(leftArmAngle), (int(positions["left_elbow"][0]), int(positions["left_elbow"][1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, str(rightArmAngle), (int(positions["right_elbow"][0]), int(positions["right_elbow"][1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, str(leftShoulderAngle), (int(positions["left_shoulder"][0]), int(positions["left_shoulder"][1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, str(rightShoulderAngle), (int(positions["right_shoulder"][0]), int(positions["right_shoulder"][1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, str(headPosition), (int(positions["nose"][0]), int(positions["nose"][1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, str(leftWristPosition), (int(positions["left_wrist"][0]), int(positions["left_wrist"][1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, str(rightWristPosition), (int(positions["right_wrist"][0]), int(positions["right_wrist"][1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    def calculateAverage(self):
        avg_data = {
            "leftArmAngle": 0,
            "rightArmAngle": 0,
            "leftShoulderAngle": 0,
            "rightShoulderAngle": 0,
            "leftWristPosition": "",
            "rightWristPosition": "",
            "headPosition": "",
            "headPositionY": 0,
            "wristDistance": 0,
        }
        num_frames = min(self.frame_queue.qsize(), 5)
        for _ in range(num_frames):
            data = self.frame_queue.get()
            for key in avg_data.keys():
                if isinstance(avg_data[key], (int, float)):
                    avg_data[key] += data[key]
                else:
                    avg_data[key] = data[key]
            # Remettre les donnÃ©es dans la file d'attente
            self.frame_queue.put(data)
        for key in avg_data.keys():
            if isinstance(avg_data[key], (int, float)):
                avg_data[key] /= num_frames
        return avg_data

    def sendAverage(self, avg_movement_data):
        # Envoyer les donnÃ©es moyennes Ã  un autre endroit (par exemple, via UDP)
        self.udpCommunicator.SendData(str(avg_movement_data))

    def run(self):
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    break

                processedFrame = self.processFrame(frame)

                if processedFrame is not None:
                    cv2.imshow('Image originale', processedFrame)

                if cv2.waitKey(1) == ord('q'):
                    break
        finally:
            self.cleanup()

    def cleanup(self):
        if hasattr(self, 'udpCommunicator') and self.udpCommunicator is not None:
            try:
                self.udpCommunicator.Close()
            except Exception as e:
                print(f"Error closing UDP communicator: {e}")

        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    poseAnalyzer = PoseAnalyzer()
    poseAnalyzer.run()
