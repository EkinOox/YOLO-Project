import cv2
import numpy as np
from ultralytics import YOLO
import threading
import UdpComms as U

class PoseAnalyzer:
    def __init__(self):
        # Initialisation du modèle YOLO et du signal de démarrage
        self.model = None
        self.startSignal = threading.Event()
        self.udpCommunicator = U.UdpComms(
            udpIP="127.0.0.1", portTX=8000, portRX=8001, enableRX=True, suppressWarnings=True)
        self.cap = cv2.VideoCapture(0)
        self.initializeYolo()

    def initializeYolo(self):
        # Initialiser YOLO dans un thread séparé
        self.initializeYoloThread()

    def initializeYoloThread(self):
        # Initialiser le modèle YOLO
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
    def getHeadInclination(positions):
        leftSide = PoseAnalyzer.calculateDistance(
            positions["left_ear"], positions["left_shoulder"])
        rightSide = PoseAnalyzer.calculateDistance(
            positions["right_ear"], positions["right_shoulder"])
        return "gauche" if leftSide > rightSide else "droite" if leftSide < rightSide else "centre"

    def processFrame(self, frame):
        results = self.model(frame, conf=0.6)
        f = results[0].plot()
        keypoints = results[0].keypoints.xy.tolist()
        peoples = list()
    
        #mise en place de position de base pour éviter de faire planter le script si aucune personne
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
    
        #récupèration des positions des points liés à Yolo
        for person in keypoints:
            if not person:
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
    
            # Utilise la premiére personne détectée pour simplifier l'exemple
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
            headInclination = PoseAnalyzer.getHeadInclination(positions)
        
            movement_data = {
                "leftArmAngle": leftArmAngle,
                "rightArmAngle": rightArmAngle,
                "leftShoulderAngle": leftShoulderAngle,
                "rightShoulderAngle": rightShoulderAngle,
            }
        
            self.drawAngles(f, positions, leftArmAngle, rightArmAngle,
                            leftShoulderAngle, rightShoulderAngle, headInclination)
        
            self.udpCommunicator.SendData(str(movement_data))
        
            return f

    @staticmethod
    def drawAngles(frame, positions, leftArmAngle, rightArmAngle, leftShoulderAngle, rightShoulderAngle, headInclination):
        cv2.putText(frame, str(leftArmAngle), (int(positions["left_elbow"][0]), int(positions["left_elbow"][1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, str(rightArmAngle), (int(positions["right_elbow"][0]), int(positions["right_elbow"][1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, str(leftShoulderAngle), (int(positions["left_shoulder"][0]), int(positions["left_shoulder"][1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, str(rightShoulderAngle), (int(positions["right_shoulder"][0]), int(positions["right_shoulder"][1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, str(headInclination), (int(positions["nose"][0]), int(positions["nose"][1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

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
