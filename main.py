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
    #calibracion = Calibracion()
    #matrix, dist = calibracion.calibracion_cam()
    matrix = np.array([[1424, 0, 550.01],
                          [0, 14553, 82.193],
                          [0, 0, 1]])
    dist = np.array([[-0.15325, 7.0021, -0.0070403, 0.10798, -29.829]])

    print("Matriz de la cámara: ", matrix)
    print("Coeficiente de Distorsión ", dist)

    # Inicializamos la cámara
    cap = cv2.VideoCapture(1)
    width = int(cap.get(3))
    height = int(cap.get(4))
    font = cv2.FONT_HERSHEY_PLAIN
    starting_time = time.time()
    frame_id = 0

    # Agregar una lista para almacenar las escalas correspondientes a cada ArUco
    aruco_scales = None  # Modificar según la cantidad de ArUcos

    while True:
        ret, frame = cap.read()

        # Detección de ArUco
        aruco_detector.detect_markers(frame, matrix, dist)

        # Actualizar las escalas y distancias para cada ArUco
        aruco_scale = aruco_detector.get_aruco_scale()
        if aruco_scale is not None:
            aruco_scales = aruco_scale

        # Detección de YOLO
        annotated_frame, boxes = yolo_detector.detect_objects(frame)

        for box in boxes:
            x, y, w, h = box.xywh[0]

            x1 = x
            y1 = y
            obj_center = [int(x1.item()), int(y1.item())]

            # Dibujar línea entre el centro del ArUco seleccionado y los objetos detectados
            selected_aruco_id = 0  # Modificar según la ID del ArUco que deseas seguir
            aruco_center = aruco_detector.get_aruco_center()

            if aruco_center is not None:
                aruco_center = (int(aruco_center[0]), int(aruco_center[1]))

            # Calcular y mostrar la distancia en el video
                distance_cm = aruco_detector.calculate_distance(int(obj_center[0]), int(obj_center[1]), annotated_frame)
                if distance_cm is not None:
                    cv2.putText(annotated_frame, f"{distance_cm:.2f} cm", obj_center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255),
                            2)


            cv2.line(annotated_frame, aruco_center, obj_center, (0, 255, 0), 2)

        # Mostramos el frame resultante
        cv2.imshow("Combined Detection", annotated_frame)


        # Oprimiendo la tecla "q" finalizamos el proceso
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
