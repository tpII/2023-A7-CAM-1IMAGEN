# main.py
import cv2
import numpy as np
from calibracion import *
from aruco_detection import ArucoDetector
from yolo_detection import YoloDetector
import time

def main():
    # Inicializamos el detector de ArUco
    aruco_detector = ArucoDetector()

    # Inicializamos el detector de YOLO
    yolo_detector = YoloDetector(model_path='yolov8n.pt')  # Especifica la ruta correcta a tu modelo YOLO

    # Calibración de la cámara
    calibracion = Calibracion()
    matrix = np.array([[1.03545930e+03, 0.00000000e+00, 5.31247578e+02],
                       [0.00000000e+00, 9.97638715e+02, 2.44565220e+02],
                       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    dist = np.array([[-0.08522228, 1.17506412, -0.01180236, 0.0784548, -2.5872603]])
    print("Matriz de la cámara: ", matrix)
    print("Coeficiente de Distorsión ", dist)

    # Inicializamos la cámara
    cap = cv2.VideoCapture(1    )
    width = int(cap.get(3))
    height = int(cap.get(4))
    font = cv2.FONT_HERSHEY_PLAIN
    starting_time = time.time()
    frame_id = 0

    while True:
        ret, frame = cap.read()

        # Detección de ArUco
        aruco_detector.detect_markers(frame, matrix, dist)

        # Detección de YOLO
        yolo_detector.detect_objects(frame, width, height, frame_id, starting_time)

        # Dibujar línea entre el centro del ArUco y los objetos detectados
        aruco_center = aruco_detector.get_aruco_center()
        obj_centers = yolo_detector.get_object_centers()

        if aruco_center is not None and obj_centers:
            aruco_center = (int(aruco_center[0]), int(aruco_center[1]))

            for obj_center in obj_centers:
                obj_center = (int(obj_center[0]), int(obj_center[1]))

                # Dibujar la línea
                cv2.line(frame, aruco_center, obj_center, (0, 255, 0), 2)

                # Calcular y mostrar la distancia en el video
                distance_cm = aruco_detector.calculate_distance(obj_center[0])
                if distance_cm is not None:
                    cv2.putText(frame, f"{distance_cm:.2f} cm", obj_center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Mostramos el frame resultante
        cv2.imshow("Combined Detection", frame)

        # Oprimiendo la tecla "q" finalizamos el proceso
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
