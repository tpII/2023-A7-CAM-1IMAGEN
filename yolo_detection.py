# yolo_detection.py
import cv2
import numpy as np
from ultralytics import YOLO
import time

class YoloDetector:
    def __init__(self, model_path='yolov8n.pt'):
        self.model = YOLO(model_path)
        self.detected_objects = []

    def detect_objects(self, frame):
        # Run YOLOv8 tracking on the frame, persisting tracks between frames
        results = self.model.track(frame, persist=True)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        boxes = results[0].boxes
        #print("BOXES", boxes)

        return annotated_frame, boxes


    def get_object_centers(self):
        return [(obj[0], obj[1]) for obj in self.detected_objects]


    def get_detected_objects(self):
        return self.detected_objects
