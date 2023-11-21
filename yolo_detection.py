# yolo_detection.py
import cv2
import numpy as np
from ultralytics import YOLO
import time

class YoloDetector:
    def __init__(self, model_path='yolov8n.pt'):
        self.model = YOLO(model_path)
        self.detected_objects = []

    def detect_objects(self, frame, width, height, frame_id, starting_time):
        # Realiza la detección con YOLOv8
        results = self.model(frame)

        # Asegúrate de que results es una lista
        if not isinstance(results, list):
            results = [results]

        # Itera sobre los resultados para cada imagen
        for result in results:
            # Modifica esta línea para acceder a las coordenadas de las cajas a través de xyxy
            boxes = result.boxes.xyxy.cpu().numpy()
            confidences = result.boxes.conf.cpu().numpy()
            class_ids = result.boxes.cls.cpu().numpy().astype(int)

            # Filtra las cajas basándote en el umbral de confianza
            mask = confidences > 0.2
            boxes = boxes[mask]
            class_ids = class_ids[mask]

            self.detected_objects = []
            for i, box in enumerate(boxes):
                x, y, w, h = box[:4]
                self.detected_objects.append((int(x), int(y), int(w), int(h)))

            # Dibuja los cuadros delimitadores en el frame
            for i, box in enumerate(boxes):
                x, y, w, h = box[:4]
                label = self.model.names[class_ids[i]]
                confidence = confidences[i]
                color = (0, 255, 0)  # Puedes establecer tu color deseado
                cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), color, 2)
                cv2.putText(frame, f"{label} {confidence:.2f}", (int(x), int(y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Tu código existente para calcular FPS
        elapsed_time = time.time() - starting_time
        fps = frame_id / elapsed_time
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 0), 3)

    def get_object_centers(self):
        return [(obj[0] + obj[2] // 2, obj[1] + obj[3] // 2) for obj in self.detected_objects]

    def get_detected_objects(self):
        return self.detected_objects
