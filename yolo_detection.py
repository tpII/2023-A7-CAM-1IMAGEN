# yolo_detection.py
import cv2
import numpy as np
import time

class YoloDetector:
    def __init__(self):
        self.model = cv2.dnn.readNet("C:\\Users\\CreZ#\\Desktop\\2023-A7-CAM-1IMAGEN\\darknet\\yolov3-tiny.weights",
                                     "C:\\Users\\CreZ#\\Desktop\\2023-A7-CAM-1IMAGEN\\darknet\\cfg\\yolov3-tiny.cfg")
        self.classes = []
        with open("C:\\Users\\CreZ#\\Desktop\\2023-A7-CAM-1IMAGEN\\darknet\\data\\coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.layer_names = self.model.getLayerNames()
        self.output_layers = [self.layer_names[i - 1] for i in self.model.getUnconnectedOutLayers()]
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))
        self.detected_objects = []

    def get_object_centers(self):
        centers = []
        for obj in self.detected_objects:
            x, y, w, h = obj
            center_x = x + w // 2
            center_y = y + h // 2
            centers.append((center_x, center_y))
        return centers

    def detect_objects(self, frame, width, height, frame_id, starting_time):
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.model.setInput(blob)
        outs = self.model.forward(self.output_layers)
        boxes = []
        confidences = []
        class_ids = []

        font = cv2.FONT_HERSHEY_PLAIN

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.2:
                    # Objeto detectado
                    center_x = int(detection[0]* width)
                    center_y = int(detection [1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    # Coordenadas del rectángulo
                    x = int(center_x - w/2)
                    y = int(center_y - h/2)
                    boxes.append([x, y, w, h])
                    confidences.append(float (confidence) )
                    class_ids.append(class_id)
        #self.detected_objects = boxes
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.4, 0.3)
        self.detected_objects = []
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                self.detected_objects.append((x, y, w, h))
        for i in range(len(boxes)):
            if i in indexes:
                x,y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                confidence = confidences[i]
                color = self.colors[class_ids[i]]
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                cv2.rectangle(frame, (x, y), (x + w, y + 30), color, -1)
                cv2.putText(frame, label + " " + str(round(confidence, 2)), (x, y + 30), font, 3, (255,255,255), 3)
        elapsed_time = time.time() - starting_time
        fps = frame_id / elapsed_time
        cv2.putText(frame, "FPS: " + str(round(fps, 2)), (10, 50), font, 3, (0, 0, 0), 3)

    def get_detected_objects(self):
        return self.detected_objects
